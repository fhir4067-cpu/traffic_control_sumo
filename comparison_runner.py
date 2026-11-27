"""Run reproducible BASELINE and ADAPTIVE CCTV1 SUMO comparisons."""

import argparse
import csv
from datetime import datetime
import json
import random
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import requests
import traci

import bridge as cctv1


PROJECT_DIR = Path(__file__).resolve().parent
SCENARIO_FILE = PROJECT_DIR / "data" / "comparison_scenario.json"
RESULTS_DIR = PROJECT_DIR / "data" / "comparison_results"
SUMO_GUI = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
SUMO_CLI = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo.exe"
TLS_ID = cctv1.TLS_ID
STEP_LENGTH = 1.0
INITIAL_QUEUE_ROUTE_PREFIX = "comparison_initial_east"
EMERGENCY_TYPE = "comparison_emergency_red"
EMERGENCY_COLOR = (255, 0, 0, 255)


def _load_scenario(path):
    scenario = json.loads(path.read_text(encoding="utf-8"))
    duration = float(scenario["duration_seconds"])
    warmup = float(scenario["warmup_seconds"])
    if duration <= 0 or warmup < 0 or warmup >= duration:
        raise ValueError("scenario duration must be positive and warmup must be in [0, duration)")

    emergency = scenario.get("emergency")
    if emergency is not None:
        direction = str(emergency["direction"]).lower()
        movement = str(emergency["movement"]).lower()
        depart = float(emergency["depart_seconds"])
        if direction not in cctv1.APPROACH_EDGES:
            raise ValueError("scenario.emergency.direction must be north/south/east/west")
        if movement not in cctv1.MOVEMENT_ROUTE_EDGES[direction]:
            raise ValueError("scenario emergency movement is invalid for its direction")
        if depart < 0 or depart >= duration:
            raise ValueError("emergency depart_seconds must be within the simulation duration")
        emergency["direction"] = direction
        emergency["movement"] = movement
    return scenario


def _validated_backend_batch(payload):
    if not isinstance(payload, dict):
        return None, "response must be a JSON object"
    if payload.get("intersection_id") != cctv1.INTERSECTION_ID:
        return None, "intersection_id mismatch"
    if str(payload.get("status", "")).upper() != "ACTIVE":
        return None, "status is not ACTIVE"
    result = payload.get("result")
    if not isinstance(result, dict):
        return None, "result must be an object"
    counts, movement_counts, counts_error = cctv1._parse_direction_counts(payload.get("data"))
    if counts_error:
        return None, counts_error
    event_state = str(result.get("event_state", "")).upper()
    priority_direction = str(result.get("priority_direction", "")).lower()
    emergency_vehicle_id = result.get("emergency_vehicle_id")
    valid_emergency = (
        event_state == "ACTIVE"
        and priority_direction in cctv1.APPROACH_EDGES
        and isinstance(emergency_vehicle_id, str)
        and bool(emergency_vehicle_id.strip())
    )
    if sum(counts.values()) <= 0 and not valid_emergency:
        return None, "all direction counts are zero"
    green_times, timing_error = cctv1._parse_green_times(result)
    if timing_error:
        return None, timing_error
    return {
        "payload": payload,
        "counts": counts,
        "movement_counts": movement_counts,
        "green_times": green_times,
        "event_state": event_state,
        "priority_direction": priority_direction,
        "emergency_vehicle_id": emergency_vehicle_id,
    }, None


def _scenario_for_backend_batch(base_scenario, batch):
    scenario = json.loads(json.dumps(base_scenario))
    scenario["traffic"] = batch["movement_counts"]
    if (
        batch["event_state"] == "ACTIVE"
        and batch["priority_direction"] in cctv1.APPROACH_EDGES
        and isinstance(batch["emergency_vehicle_id"], str)
        and batch["emergency_vehicle_id"].strip()
    ):
        depart_seconds = 0.0
        if base_scenario.get("emergency"):
            depart_seconds = float(base_scenario["emergency"]["depart_seconds"])
        scenario["emergency"] = {
            "depart_seconds": depart_seconds,
            "vehicle_id": batch["emergency_vehicle_id"],
            "direction": batch["priority_direction"],
            "movement": "straight",
        }
    else:
        scenario["emergency"] = None
    return scenario


def _build_spawn_plan(scenario):
    window = float(scenario["spawn_window_seconds"])
    rng = random.Random(int(scenario["seed"]))
    plan = []
    for direction in cctv1.APPROACH_EDGES:
        assignments = []
        for movement in cctv1.MOVEMENT_ROUTE_EDGES[direction]:
            assignments.extend([movement] * scenario["traffic"][direction][movement])
        rng.shuffle(assignments)
        count = len(assignments)
        for index, movement in enumerate(assignments):
            depart = 0.0 if count <= 1 else window * index / count
            vehicle_id = f"comparison_{direction}_{movement}_{index + 1:04d}"
            plan.append((depart, direction, movement, vehicle_id))
    return sorted(plan)


def _write_detector_additional_file(path, lanearea_output):
    root = ET.Element("additional")
    edge_lane_counts = {"1i": 4, "2i": 5, "3i": 3, "4i": 4}
    for edge, lane_count in edge_lane_counts.items():
        for lane_index in range(lane_count):
            ET.SubElement(
                root,
                "laneAreaDetector",
                {
                    "id": f"e2_{edge}_{lane_index}",
                    "lane": f"{edge}_{lane_index}",
                    "pos": "0",
                    "length": "90",
                    "freq": "1",
                    "file": str(lanearea_output.resolve()),
                    "friendlyPos": "true",
                },
            )
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _add_routes():
    for direction, movements in cctv1.MOVEMENT_ROUTE_EDGES.items():
        for movement, edges in movements.items():
            traci.route.add(f"backend_route_{direction}_{movement}", list(edges))
            if direction == "east":
                traci.route.add(f"{INITIAL_QUEUE_ROUTE_PREFIX}_{movement}", list(edges))


def _stage_initial_queue(scenario):
    queue = scenario["initial_queue"]
    direction = str(queue["direction"]).lower()
    if direction != "east":
        raise ValueError("this CCTV1 initial queue fixture currently supports the east approach")
    edge = cctv1.APPROACH_EDGES[direction]
    lane_count = traci.edge.getLaneNumber(edge)
    vehicles_per_lane = int(queue["vehicles_per_lane"])
    if lane_count != cctv1.INITIAL_QUEUE_LANES:
        raise RuntimeError(f"{edge} lane count {lane_count} does not match fixture configuration")
    approach_length = traci.lane.getLength(f"{edge}_0")
    ids = []
    for car_index in range(vehicles_per_lane):
        for lane_index in range(lane_count):
            movement = cctv1.INITIAL_EAST_LANE_MOVEMENTS[lane_index]
            position = (
                approach_length
                - float(queue["front_gap_m"])
                - car_index * float(queue["spacing_m"])
            )
            if position <= 0:
                raise RuntimeError("initial east queue does not fit on the approach")
            vehicle_id = f"comparison_initial_east_lane{lane_index}_{car_index + 1:02d}"
            traci.vehicle.add(
                vehicle_id,
                f"{INITIAL_QUEUE_ROUTE_PREFIX}_{movement}",
                typeID="DEFAULT_VEHTYPE",
                depart="now",
                departLane=str(lane_index),
                departPos="base",
                departSpeed="0",
            )
            traci.vehicle.moveTo(vehicle_id, f"{edge}_{lane_index}", position)
            traci.vehicle.setSpeed(vehicle_id, 0)
            ids.append(vehicle_id)
    if set(ids) - set(traci.vehicle.getIDList()):
        raise RuntimeError("could not stage all fixed initial-queue vehicles")
    print(
        f"[scenario] staged {lane_count} lanes x {vehicles_per_lane} vehicles "
        f"({len(ids)} total) at t={traci.simulation.getTime():.1f}s"
    )
    return ids


def _spawn_emergency_vehicle(scenario):
    emergency = scenario["emergency"]
    backend_vehicle_id = str(emergency["vehicle_id"])
    vehicle_id = cctv1._sumo_emergency_id(backend_vehicle_id)
    if EMERGENCY_TYPE not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", EMERGENCY_TYPE)
    traci.vehicletype.setColor(EMERGENCY_TYPE, EMERGENCY_COLOR)
    route_id = f"backend_route_{emergency['direction']}_{emergency['movement']}"
    traci.vehicle.add(
        vehicle_id,
        route_id,
        typeID=EMERGENCY_TYPE,
        depart="now",
        departLane="best",
        departPos="base",
        departSpeed="0",
    )
    print(
        f"[SUMO] fixed emergency spawn requested at t={traci.simulation.getTime():.1f}s: "
        f"backend_id={backend_vehicle_id}, sumo_id={vehicle_id}, "
        f"route={emergency['direction'].upper()} -> "
        f"{emergency['movement'].upper()}"
    )
    return vehicle_id


def _queue_counts():
    return {
        direction: sum(
            traci.lane.getLastStepHaltingNumber(f"{edge}_{lane_index}")
            for lane_index in range(traci.edge.getLaneNumber(edge))
        )
        for direction, edge in cctv1.APPROACH_EDGES.items()
    }


def _poll_backend(session):
    try:
        response = session.get(cctv1.SIGNAL_CONTROL_URL, timeout=1)
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            raise ValueError("response must be a JSON object")
        return payload, None
    except (requests.RequestException, ValueError) as exc:
        return None, str(exc)


def _backend_data_signature(payload):
    data = payload.get("data") if isinstance(payload, dict) else None
    batch_id = None
    if isinstance(payload, dict):
        batch_id = payload.get("batch_id", payload.get("frame_id"))
    if batch_id is None and isinstance(data, dict):
        batch_id = data.get("batch_id", data.get("frame_id"))
    count_fields = [
        f"{direction}_cars"
        for direction in cctv1.APPROACH_EDGES
    ] + [
        f"{direction}_{movement}_cars"
        for direction, movements in cctv1.MOVEMENT_ROUTE_EDGES.items()
        for movement in movements
    ]
    counts = {key: data.get(key) for key in count_fields} if isinstance(data, dict) else data
    return json.dumps({"batch_id": batch_id, "counts": counts}, sort_keys=True, ensure_ascii=False)


def _wait_for_backend_batch(
    session,
    previous_data_signature=None,
    accept_current=False,
    idle_timeout=None,
):
    """Capture one new, validated backend vehicle batch without advancing SUMO."""
    previous_active_data_signature = None
    saw_waiting = False
    last_status = None
    last_error = None
    idle_started = time.monotonic()
    print(
        "[backend] waiting for a fresh intersection 1 ACTIVE vehicle batch; "
        "SUMO has not started and simulation time remains at 0"
    )
    while True:
        payload, error = _poll_backend(session)
        if error:
            if error != last_error:
                print(f"[backend] not reachable or invalid response: {error}")
                last_error = error
            time.sleep(1)
            continue
        last_error = None
        parsed, parse_error = _parse_adaptive_snapshot(payload)
        if parse_error:
            if parse_error != last_status:
                print(f"[backend] response ignored while waiting: {parse_error}")
                last_status = parse_error
            time.sleep(1)
            continue

        status = parsed["status"] if parsed else "INVALID"
        if status == "WAITING":
            if last_status != "WAITING":
                print("[backend] WAITING; ready for the next ACTIVE vehicle batch")
            saw_waiting = True
            last_status = status
        elif status == "ACTIVE":
            batch, batch_error = _validated_backend_batch(payload)
            data_signature = _backend_data_signature(payload)
            fresh_data = saw_waiting or (
                previous_data_signature is not None
                and data_signature != previous_data_signature
            )
            fresh_initial = (
                previous_data_signature is None
                and (saw_waiting or (
                    previous_active_data_signature is not None
                    and data_signature != previous_active_data_signature
                ))
            )
            if batch_error:
                if batch_error != last_error:
                    print(f"[backend] ACTIVE batch rejected: {batch_error}")
                    last_error = batch_error
            elif accept_current or fresh_data or fresh_initial:
                trigger = "current ACTIVE response" if accept_current else "new backend vehicle batch"
                counts = batch["counts"]
                print(f"[backend] {trigger}; captured counts:", json.dumps(counts, ensure_ascii=False))
                print("[backend] captured movement counts:", json.dumps(batch["movement_counts"], ensure_ascii=False))
                return batch
            elif previous_data_signature is None and previous_active_data_signature is None:
                print(
                    "[backend] ACTIVE already existed at launch; waiting for WAITING -> ACTIVE "
                    "or changed vehicle data to avoid replaying an old batch"
                )
            previous_active_data_signature = data_signature
            last_status = status
        else:
            if last_status != status:
                print(f"[backend] status={status or '(missing)'}; waiting for ACTIVE")
                last_status = status
        if idle_timeout is not None and time.monotonic() - idle_started >= idle_timeout:
            print(
                f"[backend] no new vehicle batch for {idle_timeout:g}s; "
                "comparison runner is stopping with no vehicles active"
            )
            return None
        time.sleep(1)


def _parse_adaptive_snapshot(payload):
    if payload.get("intersection_id") != cctv1.INTERSECTION_ID:
        return None, "intersection_id mismatch"
    status = str(payload.get("status", "")).upper()
    result = payload.get("result")
    if status != "ACTIVE" or not isinstance(result, dict):
        return {"status": status, "result": result if isinstance(result, dict) else {}}, None
    green_times, timing_error = cctv1._parse_green_times(result)
    return {
        "status": status,
        "result": result,
        "green_times": green_times,
        "timing_error": timing_error,
        "event_state": str(result.get("event_state", "")).upper(),
        "priority_direction": str(result.get("priority_direction", "")).lower(),
        "emergency_vehicle_id": result.get("emergency_vehicle_id"),
    }, None


def _sumarize_tripinfo(path, warmup_seconds):
    if not path.exists():
        return {"completed": 0, "after_warmup": 0, "mean_duration": None,
                "mean_time_loss": None, "mean_waiting_time": None}
    trips = ET.parse(path).getroot().findall("tripinfo")
    scored = [trip for trip in trips if float(trip.get("depart", "0")) >= warmup_seconds]

    def mean(field):
        values = [float(trip.get(field, "0")) for trip in scored]
        return sum(values) / len(values) if values else None

    return {
        "completed": len(trips),
        "after_warmup": len(scored),
        "mean_duration": mean("duration"),
        "mean_time_loss": mean("timeLoss"),
        "mean_waiting_time": mean("waitingTime"),
        "total_time_loss": sum(float(trip.get("timeLoss", "0")) for trip in scored),
        "total_waiting_time": sum(float(trip.get("waitingTime", "0")) for trip in scored),
    }


def _summarize_lanearea(path, warmup_seconds=0.0):
    output = {}
    if not path.exists():
        return output
    root = ET.parse(path).getroot()
    for interval in root.iter("interval"):
        if float(interval.get("begin", "0")) < warmup_seconds:
            continue
        detector_id = interval.get("id", "")
        if not detector_id.startswith("e2_"):
            continue
        lane_id = detector_id[3:]
        values = output.setdefault(lane_id, {
            "entered": 0, "haltingDurationSum": 0.0, "weightedTimeLoss": 0.0,
        })
        entered = int(float(interval.get("nVehEntered", "0")))
        values["entered"] += entered
        values["haltingDurationSum"] += float(interval.get("haltingDurationSum", "0"))
        mean_loss = interval.get("meanTimeLoss")
        if mean_loss is not None:
            values["weightedTimeLoss"] += float(mean_loss) * entered
    return {
        lane_id: {
            "meanTimeLoss": values["weightedTimeLoss"] / values["entered"]
            if values["entered"] else 0.0,
            "haltingDurationSum": values["haltingDurationSum"],
            "nVehEntered": values["entered"],
        }
        for lane_id, values in output.items()
    }


def _write_timeseries_header(writer):
    writer.writerow([
        "simulation_time_s", "after_warmup", "north_halting", "east_halting",
        "south_halting", "west_halting", "total_halting",
        "cumulative_stopped_vehicle_seconds",
    ])


def _run_mode(mode, scenario, batch, batch_index, warmup, duration, output_root, run_gui, gui_delay_ms):
    output_dir = output_root / f"batch_{batch_index:03d}" / mode.lower()
    output_dir.mkdir(parents=True, exist_ok=True)
    lanearea_path = output_dir / f"lanearea_{mode.lower()}.xml"
    detector_file = output_dir / f"detectors_{mode.lower()}.add.xml"
    tripinfo_path = output_dir / f"tripinfo_{mode.lower()}.xml"
    timeseries_path = output_dir / f"timeseries_{mode.lower()}.csv"
    summary_path = output_dir / f"summary_{mode.lower()}.json"
    _write_detector_additional_file(detector_file, lanearea_path)

    binary = SUMO_GUI if run_gui else SUMO_CLI
    command = [
        binary,
        "-n", str(cctv1.NETWORK_FILE),
        "--additional-files", str(detector_file),
        "--begin", "0",
        "--end", str(duration),
        "--step-length", str(STEP_LENGTH),
        "--seed", str(scenario["seed"]),
        "--tripinfo-output", str(tripinfo_path),
        "--no-step-log", "true",
    ]
    if run_gui:
        command.extend(["--start", "--delay", str(gui_delay_ms)])

    print(f"=== CCTV1 {mode} comparison run ===")
    print(f"[scenario] duration={duration:g}s; warmup={warmup:g}s; seed={scenario['seed']}")
    print(
        "[scenario] SUMO demand comes from the captured backend movement counts; "
        "the same captured batch is reused for both modes"
    )
    print("[scenario] captured counts:", json.dumps(batch["counts"], ensure_ascii=False))
    print("[scenario] captured movement counts:", json.dumps(batch["movement_counts"], ensure_ascii=False))
    print(f"[SUMO] network: {cctv1.NETWORK_FILE}")
    traci.start(command, verbose=True)

    session = requests.Session()
    session.headers.update({"ngrok-skip-browser-warning": "true"})
    controller = {
        "state": "IDLE", "direction": None, "vehicle_id": None,
        "resume_phase": None, "deadline": None,
    }
    initial_queue_ids = []
    initial_queue_released = False
    spawn_plan = _build_spawn_plan(scenario)
    backend_armed = False
    last_backend_signature = None
    last_count_signature = None
    backend_event_state = batch["event_state"] or "NONE"
    backend_priority_direction = batch["priority_direction"] or None
    backend_emergency_id = batch["emergency_vehicle_id"]
    active_green_times = batch["green_times"] if mode == "ADAPTIVE" else None
    pending_clear_green_times = None
    emergency_vehicle_id = None
    emergency_spawned = False
    emergency_arrived = False
    emergency_clear_post_succeeded = False
    clear_post_last_attempt = float("-inf")
    early_cleared_logged = False
    direction_mismatch_logged = False
    backend_error_logged = None
    last_phase = None
    cumulative_stopped_vehicle_seconds = 0.0
    cumulative_all_stopped_vehicle_seconds = 0.0
    total_arrived = 0
    total_arrived_after_warmup = 0
    batch_completed = False
    session_control_signatures = set()

    try:
        _add_routes()
        initial_queue_ids = _stage_initial_queue(scenario)
        previous_roads = {
            vehicle_id: traci.vehicle.getRoadID(vehicle_id)
            for vehicle_id in initial_queue_ids
        }
        session_control_signatures = set()
        with timeseries_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            _write_timeseries_header(writer)

            while traci.simulation.getTime() < duration:
                sim_time = traci.simulation.getTime()
                snapshot = None
                if mode == "ADAPTIVE":
                    payload, backend_error = _poll_backend(session)
                    if backend_error:
                        if backend_error != backend_error_logged:
                            print(f"[backend] adaptive poll failed: {backend_error}")
                            backend_error_logged = backend_error
                    elif payload is not None:
                        backend_error_logged = None
                        parsed, parse_error = _parse_adaptive_snapshot(payload)
                        if parse_error:
                            print(f"[backend] adaptive response ignored: {parse_error}")
                        elif parsed and parsed["status"] == "WAITING":
                            if not backend_armed:
                                print("[backend] WAITING; armed for fresh adaptive control")
                            backend_armed = True
                            last_backend_signature = None
                        elif parsed and parsed["status"] == "ACTIVE":
                            result = parsed["result"]
                            signature = json.dumps(payload, sort_keys=True, ensure_ascii=False)
                            changed = last_backend_signature is not None and signature != last_backend_signature
                            # GET latest is the only control source for this run.
                            # Use its first ACTIVE snapshot as the run's initial
                            # control, then accept later changed snapshots.
                            fresh = backend_armed or changed or last_backend_signature is None
                            if last_backend_signature is None:
                                print("[backend] applying the current ACTIVE snapshot as the initial adaptive control")
                            last_backend_signature = signature
                            if fresh:
                                backend_armed = False
                                event_state = parsed["event_state"]
                                backend_event_state = event_state or "NONE"
                                backend_priority_direction = parsed["priority_direction"] or None
                                backend_emergency_id = parsed["emergency_vehicle_id"]
                                control_signature = json.dumps(
                                    {
                                        "event_state": backend_event_state,
                                        "mode": result.get("mode"),
                                        "priority_direction": result.get("priority_direction"),
                                        "emergency_vehicle_id": backend_emergency_id,
                                        "active_events": result.get("active_events", []),
                                        "green_times": result.get("green_times"),
                                    },
                                    sort_keys=True,
                                    ensure_ascii=False,
                                )
                                if control_signature not in session_control_signatures:
                                    print("[backend] adaptive control:", control_signature)
                                    session_control_signatures.add(control_signature)
                                counts = payload.get("data")
                                if isinstance(counts, dict):
                                    count_keys = (
                                        "north_cars", "south_cars", "east_cars", "west_cars"
                                    )
                                    count_snapshot = json.dumps(
                                        {key: counts.get(key) for key in count_keys},
                                        sort_keys=True,
                                        ensure_ascii=False,
                                    )
                                    if count_snapshot != last_count_signature:
                                        print("[backend] observed direction counts:", count_snapshot)
                                        last_count_signature = count_snapshot
                                if parsed["timing_error"]:
                                    print(f"[backend] adaptive green_times unavailable: {parsed['timing_error']}")
                                elif parsed["green_times"] is not None:
                                    if event_state == "CLEARED":
                                        pending_clear_green_times = parsed["green_times"]
                                    elif event_state != "ACTIVE" and active_green_times is None:
                                        active_green_times = parsed["green_times"]
                                snapshot = parsed

                if not initial_queue_released:
                    for vehicle_id in initial_queue_ids:
                        traci.vehicle.setSpeed(vehicle_id, -1)
                    initial_queue_released = True

                spawned = 0
                while spawn_plan and spawn_plan[0][0] <= sim_time:
                    _, direction, movement, vehicle_id = spawn_plan.pop(0)
                    traci.vehicle.add(
                        vehicle_id,
                        f"backend_route_{direction}_{movement}",
                        typeID="DEFAULT_VEHTYPE",
                        depart="now",
                        departLane="best",
                        departPos="base",
                        departSpeed="0",
                    )
                    spawned += 1
                if spawned:
                    print(f"[scenario] t={sim_time:.1f}s spawned {spawned} backend-count vehicles")

                emergency_config = scenario.get("emergency")
                if (
                    emergency_config is not None
                    and not emergency_spawned
                    and sim_time >= float(emergency_config["depart_seconds"])
                ):
                    emergency_vehicle_id = _spawn_emergency_vehicle(scenario)
                    emergency_spawned = True

                if mode == "ADAPTIVE" and backend_event_state == "ACTIVE" and emergency_spawned and controller["state"] == "IDLE":
                    fixture_direction = emergency_config["direction"]
                    if backend_priority_direction == fixture_direction:
                        controller.update(
                            state="IDLE", direction=None,
                            vehicle_id=backend_emergency_id,
                            resume_phase=None, deadline=None,
                        )
                        cctv1._begin_emergency_priority(
                            fixture_direction, controller, sim_time
                        )
                        print(
                            f"[SUMO] adaptive priority started for fixed emergency direction "
                            f"{fixture_direction.upper()} at t={sim_time:.1f}s"
                        )
                    elif not direction_mismatch_logged:
                        print(
                            f"[SUMO] backend priority_direction={backend_priority_direction!r} "
                            f"does not match fixed emergency approach={fixture_direction.upper()}; "
                            "not applying emergency priority"
                        )
                        direction_mismatch_logged = True

                if mode == "ADAPTIVE" and backend_event_state == "CLEARED" and controller["state"] != "IDLE":
                    if emergency_arrived and emergency_clear_post_succeeded:
                        if pending_clear_green_times is not None:
                            active_green_times = pending_clear_green_times
                        if controller["state"] != "CLEARING_TO_RESUME":
                            cctv1._begin_emergency_resume(controller, sim_time)
                            print("[SUMO] adaptive CLEARED accepted after local arrival and clear POST")
                    elif not emergency_arrived and not early_cleared_logged:
                        print("[SUMO] premature backend CLEARED ignored until fixed emergency reaches route destination")
                        early_cleared_logged = True

                emergency_override = False
                if mode == "ADAPTIVE":
                    controller_before = controller["state"]
                    emergency_override = cctv1._advance_emergency_controller(
                        controller, sim_time, active_green_times
                    )
                    if controller_before != "IDLE" and controller["state"] == "IDLE":
                        last_phase = None

                phase = traci.trafficlight.getPhase(TLS_ID)
                if (
                    mode == "ADAPTIVE"
                    and not emergency_override
                    and controller["state"] == "IDLE"
                    and traci.trafficlight.getProgram(TLS_ID) == "0"
                    and phase in cctv1.PHASE_DIRECTIONS
                    and phase != last_phase
                    and active_green_times
                ):
                    direction = cctv1.PHASE_DIRECTIONS[phase]
                    traci.trafficlight.setPhaseDuration(TLS_ID, active_green_times[direction])
                    print(
                        f"[SUMO] adaptive phase {phase} ({direction} green) uses "
                        f"{active_green_times[direction]:g}s; network yellow unchanged"
                    )

                traci.simulationStep()
                sim_time = traci.simulation.getTime()
                arrived_ids = set(traci.simulation.getArrivedIDList())
                total_arrived += len(arrived_ids)
                if sim_time > warmup:
                    total_arrived_after_warmup += len(arrived_ids)
                if emergency_vehicle_id in arrived_ids:
                    emergency_arrived = True
                    print(
                        f"[SUMO] fixed emergency reached route destination at "
                        f"t={sim_time:.1f}s: {emergency_vehicle_id}"
                    )

                if (
                    mode == "ADAPTIVE"
                    and emergency_arrived
                    and controller["state"] != "IDLE"
                    and not emergency_clear_post_succeeded
                    and sim_time - clear_post_last_attempt >= 5
                ):
                    clear_post_last_attempt = sim_time
                    emergency_clear_post_succeeded = cctv1._post_emergency_clear(
                        session, backend_emergency_id or emergency_config["vehicle_id"]
                    )

                queues = _queue_counts()
                total_halting = sum(queues.values())
                cumulative_all_stopped_vehicle_seconds += total_halting * STEP_LENGTH
                after_warmup = sim_time > warmup
                if after_warmup:
                    cumulative_stopped_vehicle_seconds += total_halting * STEP_LENGTH
                writer.writerow([
                    f"{sim_time:.1f}", int(after_warmup), queues["north"], queues["east"],
                    queues["south"], queues["west"], total_halting,
                    f"{cumulative_stopped_vehicle_seconds:.1f}",
                ])

                current_phase = traci.trafficlight.getPhase(TLS_ID)
                if current_phase in cctv1.PHASE_DIRECTIONS and current_phase != last_phase:
                    last_phase = current_phase
                if mode == "ADAPTIVE":
                    time.sleep(STEP_LENGTH)

                if (
                    not spawn_plan
                    and traci.simulation.getMinExpectedNumber() <= 0
                    and controller["state"] == "IDLE"
                ):
                    print(
                        f"[SUMO] backend batch {batch_index} completed; "
                        "no active or scheduled vehicles remain"
                    )
                    batch_completed = True
                    break

        if not batch_completed:
            print(
                f"[SUMO] maximum simulation time {duration:g}s reached before the batch fully cleared; "
                f"vehicles remaining={traci.simulation.getMinExpectedNumber()}"
            )

        arrived_total_at_end = total_arrived
    finally:
        session.close()
        traci.close()

    trip_summary = _sumarize_tripinfo(tripinfo_path, warmup)
    lane_summary = _summarize_lanearea(lanearea_path, warmup)
    with (output_dir / f"lane_summary_{mode.lower()}.csv").open(
        "w", newline="", encoding="utf-8"
    ) as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["lane", "meanTimeLoss", "haltingDurationSum", "nVehEntered"])
        for lane in sorted(lane_summary):
            values = lane_summary[lane]
            writer.writerow([
                lane, values["meanTimeLoss"], values["haltingDurationSum"], values["nVehEntered"]
            ])

    summary = {
        "mode": mode,
        "duration_seconds": duration,
        "warmup_seconds": warmup,
        "seed": scenario["seed"],
        "backend_regular_vehicles": sum(
            sum(movements.values()) for movements in scenario["traffic"].values()
        ),
        "staged_initial_queue_vehicles": (
            scenario["initial_queue"]["vehicles_per_lane"] * cctv1.INITIAL_QUEUE_LANES
        ),
        "backend_counts": batch["counts"],
        "backend_movement_counts": batch["movement_counts"],
        "backend_green_times_at_capture": batch["green_times"],
        "backend_mode_at_capture": batch["payload"].get("result", {}).get("mode"),
        "captured_emergency": scenario.get("emergency"),
        "batch_completed": batch_completed,
        "completed_tripinfo": trip_summary,
        "arrived_ids_seen": arrived_total_at_end,
        "arrived_ids_after_warmup": total_arrived_after_warmup,
        "cumulative_stopped_vehicle_seconds_after_warmup": cumulative_stopped_vehicle_seconds,
        "cumulative_stopped_vehicle_seconds_all": cumulative_all_stopped_vehicle_seconds,
        "lane_area": lane_summary,
        "outputs": {
            "tripinfo": str(tripinfo_path),
            "timeseries": str(timeseries_path),
            "lanearea": str(lanearea_path),
            "lane_summary": str(output_dir / f"lane_summary_{mode.lower()}.csv"),
        },
    }
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"[stats] tripinfo: {json.dumps(trip_summary, ensure_ascii=False)}")
    print(f"[stats] time series: {timeseries_path}")
    lane_summary_path = output_dir / f"lane_summary_{mode.lower()}.csv"
    print(f"[stats] lane summary: {lane_summary_path}")
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("baseline", "adaptive", "both"), required=True)
    parser.add_argument("--scenario", type=Path, default=SCENARIO_FILE)
    parser.add_argument("--duration", type=float, help="override scenario duration in seconds")
    parser.add_argument("--warmup", type=float, help="override warm-up exclusion in seconds")
    parser.add_argument("--gui", action="store_true", help="show SUMO-GUI instead of running headless")
    parser.add_argument(
        "--gui-delay-ms", type=int, default=1000,
        help="GUI delay per simulation step in milliseconds (default: 1000, real-time pace)",
    )
    parser.add_argument(
        "--accept-current", action="store_true",
        help="start from an ACTIVE response already present when the command launches",
    )
    parser.add_argument(
        "--idle-timeout", type=float, default=5.0,
        help="stop waiting for another vehicle batch after this many quiet seconds (default: 5)",
    )
    args = parser.parse_args()

    scenario = _load_scenario(args.scenario)
    duration = args.duration if args.duration is not None else float(scenario["duration_seconds"])
    warmup = args.warmup if args.warmup is not None else float(scenario["warmup_seconds"])
    if duration <= 0 or warmup < 0 or warmup >= duration:
        parser.error("duration must be positive and warmup must be in [0, duration)")
    if args.gui_delay_ms <= 0:
        parser.error("gui-delay-ms must be a positive number of milliseconds")
    if args.idle_timeout <= 0:
        parser.error("idle-timeout must be positive")

    modes = ("BASELINE", "ADAPTIVE") if args.mode == "both" else (args.mode.upper(),)
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_root = RESULTS_DIR / run_id
    output_root.mkdir(parents=True, exist_ok=True)
    batch_index = 1
    previous_data_signature = None
    session = requests.Session()
    session.headers.update({"ngrok-skip-browser-warning": "true"})
    try:
        while True:
            batch = _wait_for_backend_batch(
                session,
                previous_data_signature=previous_data_signature,
                accept_current=args.accept_current and batch_index == 1,
                idle_timeout=args.idle_timeout if batch_index > 1 else None,
            )
            if batch is None:
                break

            run_scenario = _scenario_for_backend_batch(scenario, batch)
            batch_dir = output_root / f"batch_{batch_index:03d}"
            batch_dir.mkdir(parents=True, exist_ok=True)
            (batch_dir / "backend_snapshot.json").write_text(
                json.dumps(batch["payload"], indent=2, ensure_ascii=False), encoding="utf-8"
            )
            summaries = {}
            print(
                f"[comparison] batch {batch_index}: baseline and adaptive will use the same "
                "captured backend vehicle batch"
            )
            for mode in modes:
                summaries[mode.lower()] = _run_mode(
                    mode, run_scenario, batch, batch_index, warmup, duration,
                    output_root, args.gui, args.gui_delay_ms,
                )
            if len(summaries) == 2:
                comparison_path = batch_dir / "comparison_summary.json"
                comparison_path.write_text(
                    json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8"
                )
                print(f"[stats] baseline/adaptive summary: {comparison_path}")

            previous_data_signature = _backend_data_signature(batch["payload"])
            batch_index += 1
            print(
                f"[comparison] batch finished; waiting up to {args.idle_timeout:g}s "
                "for additional backend vehicle data"
            )
    finally:
        session.close()


if __name__ == "__main__":
    main()
