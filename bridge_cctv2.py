"""Connect the SUMO cross demo to the backend's monitoring/control APIs."""

import json
import math
import time
from collections.abc import Mapping
from numbers import Real
from pathlib import Path
import xml.etree.ElementTree as ET

import requests
import traci


BACKEND_URL = "http://172.20.134.174:8000"
INTERSECTION_ID = 2
SIGNAL_CONTROL_URL = (
    f"{BACKEND_URL}/api/control/signal?intersection_id={INTERSECTION_ID}"
)

PROJECT_DIR = Path(__file__).resolve().parent
SUMO_CONFIG = str(PROJECT_DIR / "data" / "cctv2" / "backend_demo.sumocfg")
NETWORK_FILE = PROJECT_DIR / "data" / "cctv2" / "cross.net.xml"
SUMO_PATH = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
TLS_ID = "0"
POLL_SECONDS = 1
METRICS_INTERVAL = 30  # simulation seconds
SPAWN_WINDOW_SECONDS = 30  # spread one backend batch across the approaches

# The regenerated network assigns one TLS link to each lane connection.
# Derive the NS/EW link groups from their incoming edges when validating phases.
APPROACH_EDGES = {
    "north": "4i",
    "east": "2i",
    "south": "3i",
    "west": "1i",
}
BACKEND_ROUTE_EDGES = {
    "north": ("54o", "4i", "3o", "53i"),
    "south": ("53o", "3i", "4o", "54i"),
    "east": ("52o", "2i", "1o", "51i"),
    "west": ("51o", "1i", "2o", "52i"),
}
PHASE_LABELS = {
    0: "north/south green",
    1: "north/south yellow clearance",
    2: "east/west green",
    3: "east/west yellow clearance",
}


def _load_network_green_times():
    """Read green durations after validating the four-way, lane-expanded TLS plan."""
    root = ET.parse(NETWORK_FILE).getroot()
    logic = root.find(f"tlLogic[@id='{TLS_ID}'][@programID='0']")
    if logic is None:
        raise ValueError(f"traffic-light program 0 not found in {NETWORK_FILE}")
    phases = logic.findall("phase")
    if len(phases) != 4:
        raise ValueError("traffic-light program must retain the existing four phases")

    ns_indexes = set()
    ew_indexes = set()
    for connection in root.findall(f"connection[@tl='{TLS_ID}']"):
        link_index = connection.get("linkIndex")
        incoming_edge = connection.get("from")
        if link_index is None:
            raise ValueError("traffic-light connection is missing linkIndex")
        if incoming_edge in ("3i", "4i"):
            ns_indexes.add(int(link_index))
        elif incoming_edge in ("1i", "2i"):
            ew_indexes.add(int(link_index))
    if not ns_indexes or not ew_indexes:
        raise ValueError("could not find both north/south and east/west TLS lane groups")

    state_length = max(ns_indexes | ew_indexes) + 1

    def group_is(state, indexes, expected):
        return all(state[index].upper() == expected for index in indexes)

    def matches(phase_index, ns_state, ew_state):
        state = phases[phase_index].get("state", "")
        return (
            len(state) == state_length
            and group_is(state, ns_indexes, ns_state)
            and group_is(state, ew_indexes, ew_state)
        )

    if not (
        matches(0, "G", "R")
        and matches(1, "Y", "R")
        and matches(2, "R", "G")
        and matches(3, "R", "Y")
    ):
        raise ValueError(
            "traffic-light program must be NS green/yellow followed by EW green/yellow"
        )
    return {
        "north_south_green": float(phases[0].get("duration")),
        "east_west_green": float(phases[2].get("duration")),
    }


BASE_GREEN_TIMES = _load_network_green_times()


def _get_json(session, url, label):
    try:
        response = session.get(url, timeout=5)
        response.raise_for_status()
        payload = response.json()
        if not isinstance(payload, dict):
            print(f"[backend] invalid response: {label} must be a JSON object")
            return "INVALID_RESPONSE", None
        return "OK", payload
    except requests.exceptions.JSONDecodeError as exc:
        print(f"[backend] invalid JSON ({label}): {exc}")
        return "INVALID_JSON", None
    except requests.RequestException as exc:
        print(f"[backend] connection/request failed ({label}): {exc}")
        return "CONNECTION_FAILED", None
    except ValueError as exc:
        print(f"[backend] invalid JSON ({label}): {exc}")
        return "INVALID_JSON", None


def _parse_direction_counts(data):
    if not isinstance(data, Mapping):
        return None, "data must be an object"

    counts = {}
    for direction in APPROACH_EDGES:
        key = f"{direction}_cars"
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, Real):
            return None, f"data.{key} is missing or is not numeric"
        if not math.isfinite(float(value)) or value < 0 or not float(value).is_integer():
            return None, f"data.{key} must be a finite non-negative whole-number count"
        counts[direction] = int(value)
    return counts, None


def _parse_green_times(result):
    green_times = result.get("green_times")
    if not isinstance(green_times, Mapping):
        return None, "result.green_times is missing or is not an object"

    parsed = {}
    for key in ("north_south_green", "east_west_green"):
        value = green_times.get(key)
        if isinstance(value, bool) or not isinstance(value, Real):
            return None, f"result.green_times.{key} is missing or is not numeric seconds"
        if not math.isfinite(float(value)) or value <= 0:
            return None, f"result.green_times.{key} must be finite and greater than zero seconds"
        parsed[key] = float(value)
    return parsed, None


def get_backend_snapshot(session):
    fetch_state, control = _get_json(
        session, SIGNAL_CONTROL_URL, f"control/signal intersection_id={INTERSECTION_ID}"
    )
    if control is None:
        return fetch_state, None

    response_id = control.get("intersection_id")
    if (
        isinstance(response_id, bool)
        or not isinstance(response_id, int)
        or response_id != INTERSECTION_ID
    ):
        print(
            f"[backend] intersection_id mismatch: requested {INTERSECTION_ID}, "
            f"received {response_id!r}; response ignored"
        )
        return "INVALID_RESPONSE", {"payload": control}

    status = str(control.get("status", "")).upper()
    if status == "WAITING":
        return "WAITING", {"payload": control}

    if status != "ACTIVE":
        print(f"[backend] non-active status: {status or 'MISSING'}")
        return "INACTIVE", {"payload": control}

    counts, counts_error = _parse_direction_counts(control.get("data"))
    if counts_error:
        print(f"[backend] invalid direction counts: {counts_error}")

    result = control.get("result")
    if not isinstance(result, dict):
        print("[backend] invalid response: result must be an object")
        return "INVALID_RESPONSE", {"counts": counts, "payload": control}

    mode = str(result.get("mode", "")).upper()
    if mode != "CONGESTION_PATTERN":
        return "ACTIVE_EVENT", {
            "counts": counts or {}, "counts_error": counts_error,
            "payload": control, "result": result,
        }

    green_times, timing_error = _parse_green_times(result)
    if timing_error:
        print(f"[backend] invalid congestion timing: {timing_error}")
    if counts_error or timing_error:
        print("[backend] status:", status)
        print("[backend] direction counts:", json.dumps(counts or {}, ensure_ascii=False))
        print("[backend] congestion/mode:", result.get("mode", "MISSING"))
        print("[backend] active_events:", json.dumps(result.get("active_events", "MISSING"), ensure_ascii=False))
        print("[backend] signal_action:", json.dumps(result.get("signal_action", "MISSING"), ensure_ascii=False))
        print("[backend] green_times:", json.dumps(result.get("green_times", "MISSING"), ensure_ascii=False))
        print("[backend] message:", result.get("message", "MISSING"))
        return "INVALID_RESPONSE", {
            "counts": counts or {}, "payload": control, "result": result,
        }

    return "ACTIVE", {
        "counts": counts, "payload": control, "result": result,
        "green_times": green_times,
    }


def _green_plan_for_snapshot(state, snapshot):
    """Return green durations only for recognized backend actions."""
    if state == "ACTIVE":
        return snapshot["green_times"], "backend congestion plan"
    if state != "ACTIVE_EVENT":
        return None, None

    result = snapshot["result"]
    mode = str(result.get("mode", "")).upper()
    action = str(result.get("signal_action", "")).upper()
    if mode in ("NORMAL", "NORMAL_PATTERN") and action == "REGULAR_CYCLE":
        green_times, timing_error = _parse_green_times(result)
        if timing_error:
            print(f"[backend] regular-cycle timing unavailable: {timing_error}")
            return None, None
        return green_times, "backend REGULAR_CYCLE"
    # The backend's exit-blocked rule returns HOLD_CURRENT_SIGNAL together
    # with explicit 15/15 green_times. Preserve the phase order and yellow
    # clearances, but honor those supplied green durations in SUMO.
    if (
        mode in ("EXIT_BLOCKED_PROTECTION", "COMPLEX_CONGESTION_CONTROL")
        and action == "HOLD_CURRENT_SIGNAL"
    ):
        green_times, timing_error = _parse_green_times(result)
        if timing_error:
            print(f"[backend] exit-protection timing unavailable: {timing_error}")
            return None, None
        return green_times, "backend exit-blocked protection plan"
    return None, None


def _queue_snapshot():
    return {
        direction: sum(
            traci.lane.getLastStepHaltingNumber(f"{edge}_{lane_index}")
            for lane_index in range(traci.edge.getLaneNumber(edge))
        )
        for direction, edge in APPROACH_EDGES.items()
    }


def _build_spawn_plan(counts):
    """Create one finite set of SUMO vehicles from one backend count snapshot."""
    plan = []
    for direction, count in counts.items():
        for index in range(count):
            depart = 0.0 if count <= 1 else SPAWN_WINDOW_SECONDS * index / count
            vehicle_id = f"backend_{direction}_{index + 1:04d}"
            plan.append((depart, direction, vehicle_id))
    return sorted(plan)


def _incoming_to_green_passes(previous_roads, current_roads):
    """Count vehicles that left an approach edge for the junction this step."""
    passed = {direction: 0 for direction in APPROACH_EDGES}
    for vehicle_id, old_road in previous_roads.items():
        new_road = current_roads.get(vehicle_id)
        if new_road is None or new_road == old_road:
            continue
        for direction, edge in APPROACH_EDGES.items():
            if old_road == edge:
                passed[direction] += 1
                break
    return passed


def _print_metrics(start_queues, passed, elapsed, comparison_label):
    queues = _queue_snapshot()
    phase = traci.trafficlight.getPhase(TLS_ID)
    remaining = traci.trafficlight.getNextSwitch(TLS_ID) - traci.simulation.getTime()
    print(
        f"[SUMO] signal phase {phase} ({PHASE_LABELS.get(phase, 'unknown')}), "
        f"state={traci.trafficlight.getRedYellowGreenState(TLS_ID)}, "
        f"remaining={remaining:.1f}s"
    )
    print(f"[SUMO] {elapsed}s window ({comparison_label}) | queue start -> end, crossed stop line:")
    for direction in APPROACH_EDGES:
        print(
            f"  {direction}: {start_queues[direction]} -> {queues[direction]}, "
            f"{passed[direction]} vehicles"
        )
    return queues


def main():
    print("=== Backend to SUMO congestion demo (CCTV 2) ===")
    print(f"[SUMO] config: {SUMO_CONFIG}")
    print(f"[backend] control: GET {SIGNAL_CONTROL_URL}")

    traci.start([SUMO_PATH, "-c", SUMO_CONFIG, "--start"])
    print("[SUMO] TraCI connected")
    for direction, edges in BACKEND_ROUTE_EDGES.items():
        traci.route.add(f"backend_route_{direction}", list(edges))
    pending_spawn_plan = []

    session = requests.Session()
    # Avoid ngrok's browser interstitial when the tunnel is accessed by requests.
    session.headers.update({"ngrok-skip-browser-warning": "true"})
    last_result_signature = None
    last_data_signature = None
    last_payload_signature = None
    active_green_times = None
    run_control_latched = False
    last_phase = None
    comparison_label = "existing SUMO program (baseline)"
    previous_roads = {}
    window_passes = {direction: 0 for direction in APPROACH_EDGES}
    window_start_time = traci.simulation.getTime()
    window_start_queues = _queue_snapshot()
    last_backend_log_wall = 0.0
    last_status = None
    simulation_ready = False
    simulation_started = False
    simulation_completed = False
    waiting_logged = False
    armed_for_active = False
    stale_active_logged = False

    try:
        while True:
            if simulation_completed:
                # Keep the final SUMO frame visible and detect a user closing the GUI.
                try:
                    traci.simulation.getTime()
                except traci.exceptions.FatalTraCIError:
                    break
                time.sleep(POLL_SECONDS)
                continue

            state, snapshot = get_backend_snapshot(session)
            sim_time = traci.simulation.getTime()

            state_changed = state != last_status
            if state_changed:
                print(f"[backend] connection/status: {state}")
                last_status = state

            if state == "WAITING" and state_changed:
                print("[backend] WAITING: no control result has been published")
                if not simulation_started:
                    armed_for_active = True
                    stale_active_logged = False
                    print("[SUMO] armed; waiting for the next ACTIVE backend response")
                    active_green_times = None
                    run_control_latched = False
                    simulation_ready = False
                    last_result_signature = None
                    last_payload_signature = None
                    comparison_label = "existing SUMO program (backend WAITING)"
                else:
                    print("[SUMO] one-shot run already started; finishing with last received control")

            if state in ("ACTIVE", "ACTIVE_EVENT"):
                counts = snapshot["counts"]
                result = snapshot["result"]
                payload_signature = json.dumps(snapshot["payload"], sort_keys=True, ensure_ascii=False)
                fresh_payload = (
                    last_payload_signature is not None
                    and payload_signature != last_payload_signature
                )
                if last_payload_signature is None:
                    print(
                        f"[backend] baseline response observed at {time.strftime('%H:%M:%S')}; "
                        "waiting for a subsequent response change"
                    )
                elif fresh_payload:
                    print(
                        f"[backend] NEW response observed at {time.strftime('%H:%M:%S')} "
                        "(payload changed)"
                    )
                last_payload_signature = payload_signature

                payload_data = snapshot["payload"].get("data")
                data_signature = json.dumps(payload_data, sort_keys=True, ensure_ascii=False)
                if data_signature != last_data_signature:
                    label = "latest snapshot observed" if last_data_signature is None else "latest data changed"
                    print(f"[backend] {label} at {time.strftime('%H:%M:%S')}")
                    print("[backend] direction counts:", json.dumps(counts or {}, ensure_ascii=False))
                    last_data_signature = data_signature
                wall_now = time.monotonic()
                valid_counts = len(counts or {}) == 4

                if wall_now - last_backend_log_wall >= 5:
                    print(
                        f"[backend] GET /api/control/signal?intersection_id={INTERSECTION_ID} reachable; "
                        f"current mode={result.get('mode', 'MISSING')}"
                    )
                    last_backend_log_wall = wall_now

                if (
                    (armed_for_active or fresh_payload)
                    and valid_counts
                    and not simulation_started
                    and sum(counts.values()) > 0
                ):
                    pending_spawn_plan = _build_spawn_plan(counts)
                    simulation_ready = True
                    simulation_started = True
                    armed_for_active = False
                    stale_active_logged = False
                    waiting_logged = False
                    trigger = (
                        "ACTIVE response after WAITING"
                        if state_changed and last_status == "ACTIVE"
                        else "changed backend payload"
                    )
                    print(
                        f"[SUMO] {trigger}; creating one finite batch of "
                        f"{sum(counts.values())} vehicles from backend counts"
                    )
                    print("[SUMO] one-shot vehicle counts:", json.dumps(counts, ensure_ascii=False))
                elif (
                    (armed_for_active or fresh_payload)
                    and valid_counts
                    and sum(counts.values()) == 0
                    and not stale_active_logged
                ):
                    print("[SUMO] all backend direction counts are zero; waiting without starting")
                    stale_active_logged = True
                elif not armed_for_active and not simulation_ready and not stale_active_logged:
                    print(
                        "[SUMO] cached ACTIVE response ignored; waiting for a changed "
                        "backend payload or WAITING -> ACTIVE transition"
                    )
                    stale_active_logged = True

                signature = json.dumps(result, sort_keys=True, ensure_ascii=False)
                result_changed = signature != last_result_signature
                if result_changed or fresh_payload:
                    if result_changed:
                        print("[backend] status:", snapshot["payload"].get("status", "unknown"))
                        print("[backend] congestion/mode:", result.get("mode", "MISSING"))
                        print("[backend] active_events:", result.get("active_events", "MISSING"))
                        print("[backend] signal_action:", json.dumps(result.get("signal_action"), ensure_ascii=False))
                        print("[backend] green_times:", json.dumps(result.get("green_times", "MISSING"), ensure_ascii=False))
                        print("[backend] message:", result.get("message", "MISSING"))

                    mode = str(result.get("mode", "")).upper()
                    action = str(result.get("signal_action", "")).upper()
                    plan, plan_source = _green_plan_for_snapshot(state, snapshot)
                    if run_control_latched:
                        if plan is not None and plan != active_green_times:
                            print(
                                "[SUMO] new backend timing received during the one-shot run; "
                                "keeping the starting control until all batch vehicles finish"
                            )
                    elif plan is not None:
                        active_green_times = plan
                        comparison_label = (
                            f"{plan_source}; NS={active_green_times['north_south_green']:g}s, "
                            f"EW={active_green_times['east_west_green']:g}s"
                        )
                        if simulation_ready:
                            run_control_latched = True
                            waiting_logged = False
                            print(
                                f"[SUMO] latched {plan_source} for this one-shot batch; "
                                "applying at green phase starts and holding until completion"
                            )
                        else:
                            print(
                                f"[SUMO] {plan_source} is present in the baseline response; "
                                "waiting for a subsequent backend update"
                            )
                    elif mode == "EMERGENCY_VEHICLE_PRIORITY":
                        if simulation_ready:
                            active_green_times = None
                            waiting_logged = False
                            traci.trafficlight.setPhase(TLS_ID, 0)
                            print("[SUMO] preserved legacy emergency mapping: phase 0")
                    elif "ACCIDENT" in mode:
                        if simulation_ready:
                            active_green_times = None
                            waiting_logged = False
                            traci.trafficlight.setPhase(TLS_ID, 2)
                            print("[SUMO] preserved legacy accident mapping: phase 2")
                    elif (
                        mode == "EXIT_BLOCKED_PROTECTION"
                        and action == "HOLD_CURRENT_SIGNAL"
                    ):
                        if simulation_ready:
                            print(
                                "[SUMO] backend requested HOLD_CURRENT_SIGNAL; "
                                "keeping phase order and applying supplied green_times"
                            )
                        else:
                            print(
                                "[SUMO] HOLD_CURRENT_SIGNAL received; "
                                "waiting for a subsequent backend update before starting traffic"
                            )
                    else:
                        print(
                            f"[SUMO] no timing applied for unsupported mode/action "
                            f"{mode!r}/{action!r}"
                        )
                    last_result_signature = signature

            elif state in ("INACTIVE", "INVALID_RESPONSE", "INVALID_JSON", "CONNECTION_FAILED"):
                if state_changed:
                    if simulation_started:
                        print(
                            f"[SUMO] backend {state}; completing the started run "
                            "with the last received signal timing"
                        )
                    else:
                        print(f"[SUMO] backend {state}; remaining paused before first data")
                        active_green_times = None
                        simulation_ready = False
                        last_result_signature = None
                        last_payload_signature = None
                        comparison_label = f"existing SUMO program ({state})"

            # Keep SUMO at simulation time 0 until the backend publishes a
            # supported control result. This lets the demo show traffic starting
            # only after AI/backend control data arrives.
            if not simulation_ready:
                if not waiting_logged:
                    print("[SUMO] paused at start; waiting for a valid backend control result")
                    waiting_logged = True
                time.sleep(POLL_SECONDS)
                continue

            spawned_this_step = {direction: 0 for direction in APPROACH_EDGES}
            while pending_spawn_plan and pending_spawn_plan[0][0] <= sim_time:
                _, direction, vehicle_id = pending_spawn_plan.pop(0)
                traci.vehicle.add(
                    vehicle_id,
                    f"backend_route_{direction}",
                    typeID="DEFAULT_VEHTYPE",
                    depart="now",
                    departLane="best",
                    departPos="base",
                    departSpeed="0",
                )
                spawned_this_step[direction] += 1
            spawned_now = {d: n for d, n in spawned_this_step.items() if n}
            if spawned_now:
                print("[SUMO] released backend vehicles:", json.dumps(spawned_now))

            phase = traci.trafficlight.getPhase(TLS_ID)
            if phase in (0, 2) and phase != last_phase and active_green_times:
                if phase == 0:
                    duration = active_green_times["north_south_green"]
                    group = "north/south"
                else:
                    duration = active_green_times["east_west_green"]
                    group = "east/west"
                baseline_queues = _queue_snapshot()
                traci.trafficlight.setPhaseDuration(TLS_ID, duration)
                comparison_label = f"backend control applied to {group}; baseline queue captured"
                window_start_queues = baseline_queues
                window_passes = {direction: 0 for direction in APPROACH_EDGES}
                window_start_time = sim_time
                print(
                    f"[SUMO] applied phase {phase} ({group} green) for {duration:g}s; "
                    "yellow clearance phases remain unchanged"
                )

            traci.simulationStep()
            current_roads = {
                vehicle_id: traci.vehicle.getRoadID(vehicle_id)
                for vehicle_id in traci.vehicle.getIDList()
            }
            step_passes = _incoming_to_green_passes(previous_roads, current_roads)
            for direction in window_passes:
                window_passes[direction] += step_passes[direction]
            previous_roads = current_roads

            now = traci.simulation.getTime()
            if now - window_start_time >= METRICS_INTERVAL:
                window_start_queues = _print_metrics(
                    window_start_queues, window_passes, int(now - window_start_time), comparison_label
                )
                window_passes = {direction: 0 for direction in APPROACH_EDGES}
                window_start_time = now
                if active_green_times:
                    comparison_label = (
                        "backend timing active; "
                        f"NS={active_green_times['north_south_green']:g}s, "
                        f"EW={active_green_times['east_west_green']:g}s"
                    )
                else:
                    comparison_label = "existing SUMO program"

            if not pending_spawn_plan and traci.simulation.getMinExpectedNumber() <= 0:
                simulation_ready = False
                simulation_completed = True
                print(
                    f"[SUMO] one-shot backend batch completed at simulation time {now:.1f}s; "
                    "holding the final frame; latched signal timing was kept through completion"
                )

            last_phase = phase

            time.sleep(POLL_SECONDS)
    except KeyboardInterrupt:
        print("\n[demo] stopping")
    finally:
        session.close()
        traci.close()
        print("[SUMO] TraCI disconnected")


if __name__ == "__main__":
    main()
