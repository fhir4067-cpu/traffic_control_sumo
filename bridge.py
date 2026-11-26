"""Connect the SUMO cross demo to the backend's monitoring/control APIs."""

import json
import math
import random
import re
import time
from collections.abc import Mapping
from numbers import Real
from pathlib import Path
import xml.etree.ElementTree as ET

import requests
import traci


BACKEND_URL = "http://172.20.134.174:8000"
INTERSECTION_ID = 1
SIGNAL_CONTROL_URL = (
    f"{BACKEND_URL}/api/control/signal?intersection_id={INTERSECTION_ID}"
)

PROJECT_DIR = Path(__file__).resolve().parent
SUMO_CONFIG = str(PROJECT_DIR / "data" / "backend_demo.sumocfg")
NETWORK_FILE = PROJECT_DIR / "data" / "cross.net.xml"
SUMO_LOG_FILE = PROJECT_DIR / "data" / "cctv1_sumo.log"
SUMO_ERROR_LOG_FILE = PROJECT_DIR / "data" / "cctv1_sumo_error.log"
SUMO_PATH = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
TLS_ID = "0"
POLL_SECONDS = 1
METRICS_INTERVAL = 30  # simulation seconds
SPAWN_WINDOW_SECONDS = 30  # spread one backend batch across the approaches
INITIAL_QUEUE_EDGE = "2i"  # east approach, traveling west through the junction
INITIAL_QUEUE_LANES = 5
INITIAL_QUEUE_PER_LANE = 8
INITIAL_QUEUE_FRONT_GAP = 12.0  # leave room before the stop line
INITIAL_QUEUE_SPACING = 12.0

# The regenerated network assigns one TLS link to each lane connection.
# Separate signal-controlled left/through links from permissive right turns.
APPROACH_EDGES = {
    "north": "4i",
    "east": "2i",
    "south": "3i",
    "west": "1i",
}
MOVEMENT_ROUTE_EDGES = {
    "north": {
        "left": ("54o", "4i", "2o", "52i"),
        "straight": ("54o", "4i", "3o", "53i"),
        "right": ("54o", "4i", "1o", "51i"),
    },
    "south": {
        "left": ("53o", "3i", "1o", "51i"),
        "straight": ("53o", "3i", "4o", "54i"),
        "right": ("53o", "3i", "2o", "52i"),
    },
    "east": {
        "left": ("52o", "2i", "3o", "53i"),
        "straight": ("52o", "2i", "1o", "51i"),
        "right": ("52o", "2i", "4o", "54i"),
    },
    "west": {
        "left": ("51o", "1i", "4o", "54i"),
        "straight": ("51o", "1i", "2o", "52i"),
        "right": ("51o", "1i", "3o", "53i"),
    },
}
# SUMO lane 0 is the driver's rightmost lane. CCTV1 east lane 1 is the
# leftmost (SUMO lane 4), and east lane 5 is the rightmost (SUMO lane 0).
INITIAL_EAST_LANE_MOVEMENTS = {
    0: "right",
    1: "straight",
    2: "straight",
    3: "straight",
    4: "left",
}
GREEN_PHASES = {"east": 0, "north": 2, "south": 4, "west": 6}
PHASE_DIRECTIONS = {phase: direction for direction, phase in GREEN_PHASES.items()}
PHASE_LABELS = {
    0: "east green",
    1: "east yellow clearance",
    2: "north green",
    3: "north yellow clearance",
    4: "south green",
    5: "south yellow clearance",
    6: "west green",
    7: "west yellow clearance",
}


def _load_network_green_times():
    """Validate the direction-by-direction cycle and retain event-mode timings."""
    root = ET.parse(NETWORK_FILE).getroot()
    logic = root.find(f"tlLogic[@id='{TLS_ID}'][@programID='0']")
    if logic is None:
        raise ValueError(f"traffic-light program 0 not found in {NETWORK_FILE}")
    phases = logic.findall("phase")
    if len(phases) != 8:
        raise ValueError("traffic-light program must contain four green/yellow pairs")

    direction_indexes = {direction: set() for direction in GREEN_PHASES}
    right_turn_indexes = set()
    edge_directions = {edge: direction for direction, edge in APPROACH_EDGES.items()}
    for connection in root.findall(f"connection[@tl='{TLS_ID}']"):
        link_index = connection.get("linkIndex")
        direction = edge_directions.get(connection.get("from"))
        if link_index is None:
            raise ValueError("traffic-light connection is missing linkIndex")
        if direction is not None:
            if connection.get("dir") == "r":
                right_turn_indexes.add(int(link_index))
            else:
                direction_indexes[direction].add(int(link_index))
    if any(not indexes for indexes in direction_indexes.values()):
        raise ValueError("could not find protected TLS links for all four approaches")
    if not right_turn_indexes:
        raise ValueError("could not find permissive right-turn TLS links")

    all_indexes = right_turn_indexes | set.union(*direction_indexes.values())
    state_length = max(all_indexes) + 1
    for direction, green_phase in GREEN_PHASES.items():
        yellow_phase = green_phase + 1
        green_state = phases[green_phase].get("state", "")
        yellow_state = phases[yellow_phase].get("state", "")
        if len(green_state) != state_length or len(yellow_state) != state_length:
            raise ValueError("traffic-light phase state length does not match TLS links")
        if any(green_state[i].lower() != "g" for i in right_turn_indexes):
            raise ValueError("permissive right turns must remain in yielding-green state")
        if any(yellow_state[i].lower() != "g" for i in right_turn_indexes):
            raise ValueError("permissive right turns must remain available during clearance")
        for other_direction, indexes in direction_indexes.items():
            expected_green = "G" if other_direction == direction else "r"
            expected_yellow = "y" if other_direction == direction else "r"
            if any(green_state[i] != expected_green for i in indexes):
                raise ValueError(f"phase {green_phase} does not exclusively serve {direction}")
            if any(yellow_state[i] != expected_yellow for i in indexes):
                raise ValueError(f"phase {yellow_phase} is not a {direction} yellow clearance")

    event_logic = root.find(f"tlLogic[@id='{TLS_ID}'][@programID='legacy_events']")
    if event_logic is None or len(event_logic.findall("phase")) != 4:
        raise ValueError("legacy_events traffic-light program is required for existing event modes")
    return True


NETWORK_SIGNAL_PLAN_VALIDATED = _load_network_green_times()


def _load_network_phase_data():
    """Read the existing protected green/yellow states and clearance times."""
    root = ET.parse(NETWORK_FILE).getroot()
    logic = root.find(f"tlLogic[@id='{TLS_ID}'][@programID='0']")
    phases = logic.findall("phase")
    return (
        {index: phase.get("state", "") for index, phase in enumerate(phases)},
        {index: float(phase.get("duration", "0")) for index, phase in enumerate(phases)},
    )


TLS_PHASE_STATES, TLS_PHASE_DURATIONS = _load_network_phase_data()
EMERGENCY_EVENT_STATES = {"ACTIVE", "CLEARED"}
EMERGENCY_VEHICLE_TYPE = "emergency_red"
EMERGENCY_COLOR = (255, 0, 0, 255)
EMERGENCY_YELLOW_SECONDS = 3.0  # Emergency transitions only; normal cycle keeps net.xml timing.


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
        return None, None, "data must be an object"

    counts = {}
    movement_counts = {}
    for direction in APPROACH_EDGES:
        key = f"{direction}_cars"
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, Real):
            return None, None, f"data.{key} is missing or is not numeric"
        if not math.isfinite(float(value)) or value < 0 or not float(value).is_integer():
            return None, None, f"data.{key} must be a finite non-negative whole-number count"
        counts[direction] = int(value)

        movement_counts[direction] = {}
        for movement in MOVEMENT_ROUTE_EDGES[direction]:
            movement_key = f"{direction}_{movement}_cars"
            movement_value = data.get(movement_key)
            if isinstance(movement_value, bool) or not isinstance(movement_value, Real):
                return None, None, f"data.{movement_key} is missing or is not numeric"
            if (
                not math.isfinite(float(movement_value))
                or movement_value < 0
                or not float(movement_value).is_integer()
            ):
                return None, None, (
                    f"data.{movement_key} must be a finite non-negative whole-number count"
                )
            movement_counts[direction][movement] = int(movement_value)

        movement_total = sum(movement_counts[direction].values())
        if movement_total != counts[direction]:
            return None, None, (
                f"data.{direction}_cars={counts[direction]} does not match the sum of "
                f"{direction}_left_cars, {direction}_straight_cars, and "
                f"{direction}_right_cars ({movement_total})"
            )
    return counts, movement_counts, None


def _parse_green_times(result):
    green_times = result.get("green_times")
    if not isinstance(green_times, Mapping):
        return None, "result.green_times is missing or is not an object"

    parsed = {}
    directional_keys = {direction: f"{direction}_green" for direction in GREEN_PHASES}
    present_directional = [key in green_times for key in directional_keys.values()]
    has_directional = all(present_directional)
    if any(present_directional) and not has_directional:
        missing = [key for key in directional_keys.values() if key not in green_times]
        return None, f"directional timing is incomplete; missing {', '.join(missing)}"
    if has_directional:
        requested = {direction: green_times[key] for direction, key in directional_keys.items()}
    else:
        # Backward compatibility: split each legacy axis duration between its
        # two approaches so the old total green allocation is preserved.
        requested = {
            "north": green_times.get("north_south_green", None),
            "south": green_times.get("north_south_green", None),
            "east": green_times.get("east_west_green", None),
            "west": green_times.get("east_west_green", None),
        }
    for direction, value in requested.items():
        key = directional_keys[direction]
        if isinstance(value, bool) or not isinstance(value, Real):
            return None, f"result.green_times.{key} (or legacy axis value) is missing or not numeric seconds"
        if not math.isfinite(float(value)) or value <= 0:
            return None, f"result.green_times.{key} must be finite and greater than zero seconds"
        parsed[direction] = float(value) if has_directional else float(value) / 2.0
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

    counts, movement_counts, counts_error = _parse_direction_counts(control.get("data"))
    if counts_error:
        print(f"[backend] invalid direction counts: {counts_error}")

    result = control.get("result")
    if not isinstance(result, dict):
        print("[backend] invalid response: result must be an object")
        return "INVALID_RESPONSE", {"counts": counts, "payload": control}

    mode = str(result.get("mode", "")).upper()
    event_state = str(result.get("event_state", "")).upper()
    priority_direction = result.get("priority_direction")
    if isinstance(priority_direction, str):
        priority_direction = priority_direction.lower()
    emergency_vehicle_id = result.get("emergency_vehicle_id")
    if event_state in EMERGENCY_EVENT_STATES:
        if event_state == "ACTIVE":
            if priority_direction not in APPROACH_EDGES:
                print(
                    "[backend] invalid emergency event: result.priority_direction "
                    "must be NORTH, SOUTH, EAST, or WEST"
                )
                return "INVALID_RESPONSE", {"counts": counts or {}, "payload": control, "result": result}
            if not isinstance(emergency_vehicle_id, str) or not emergency_vehicle_id.strip():
                print("[backend] invalid emergency event: emergency_vehicle_id is missing")
                return "INVALID_RESPONSE", {"counts": counts or {}, "payload": control, "result": result}
        elif priority_direction is not None or emergency_vehicle_id is not None:
            print(
                "[backend] invalid CLEARED event: priority_direction and "
                "emergency_vehicle_id must be null"
            )
            return "INVALID_RESPONSE", {"counts": counts or {}, "payload": control, "result": result}

        active_events = result.get("active_events")
        if not isinstance(active_events, list):
            print("[backend] invalid emergency event: result.active_events must be a list")
            return "INVALID_RESPONSE", {"counts": counts or {}, "payload": control, "result": result}

        green_times, timing_error = _parse_green_times(result)
        if timing_error:
            print(f"[backend] invalid emergency timing: {timing_error}")
            return "INVALID_RESPONSE", {"counts": counts or {}, "payload": control, "result": result}
        return "ACTIVE_EVENT", {
            "counts": counts or {}, "movement_counts": movement_counts or {},
            "counts_error": counts_error, "payload": control, "result": result,
            "green_times": green_times, "event_state": event_state,
            "priority_direction": priority_direction,
            "emergency_vehicle_id": emergency_vehicle_id,
            "active_events": active_events,
        }

    if mode != "CONGESTION_PATTERN":
        return "ACTIVE_EVENT", {
            "counts": counts or {}, "movement_counts": movement_counts or {},
            "counts_error": counts_error,
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
            "counts": counts or {}, "movement_counts": movement_counts or {},
            "counts_error": counts_error, "payload": control, "result": result,
        }

    return "ACTIVE", {
        "counts": counts, "movement_counts": movement_counts,
        "payload": control, "result": result,
        "green_times": green_times, "event_state": event_state,
        "priority_direction": priority_direction,
        "emergency_vehicle_id": emergency_vehicle_id,
        "active_events": result.get("active_events", []),
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


def _build_spawn_plan(counts, movement_counts, batch_number):
    """Create a finite batch using the backend's exact per-movement counts."""
    plan = []
    for direction in APPROACH_EDGES:
        count = counts[direction]
        assignments = [
            movement
            for movement, movement_count in movement_counts[direction].items()
            for _ in range(movement_count)
        ]
        # Keep the backend-provided movement totals exact while spreading their
        # arrival order across the approach's existing spawn window.
        random.Random(f"{batch_number}:{direction}").shuffle(assignments)
        for index in range(count):
            depart = 0.0 if count <= 1 else SPAWN_WINDOW_SECONDS * index / count
            movement = assignments[index]
            vehicle_id = (
                f"backend_batch{batch_number}_{direction}_{movement}_{index + 1:04d}"
            )
            plan.append((depart, direction, movement, vehicle_id))
    return sorted(plan)


def _stage_initial_east_queue(batch_number):
    """Force-insert and hold eight vehicles per lane without advancing SUMO time."""
    if traci.edge.getLaneNumber(INITIAL_QUEUE_EDGE) != INITIAL_QUEUE_LANES:
        raise RuntimeError(
            f"{INITIAL_QUEUE_EDGE} must have {INITIAL_QUEUE_LANES} lanes for the initial queue"
        )

    approach_length = traci.lane.getLength(f"{INITIAL_QUEUE_EDGE}_0")
    staged_time = traci.simulation.getTime()
    vehicle_ids = []
    for car_index in range(INITIAL_QUEUE_PER_LANE):
        for lane_index in range(INITIAL_QUEUE_LANES):
            movement = INITIAL_EAST_LANE_MOVEMENTS[lane_index]
            user_lane_number = INITIAL_QUEUE_LANES - lane_index
            position = (
                approach_length
                - INITIAL_QUEUE_FRONT_GAP
                - car_index * INITIAL_QUEUE_SPACING
            )
            if position <= 0:
                raise RuntimeError(
                    f"{INITIAL_QUEUE_EDGE} is too short for "
                    f"{INITIAL_QUEUE_PER_LANE} staged vehicles per lane"
                )
            vehicle_id = (
                f"initial_batch{batch_number}_east_lane{user_lane_number}_"
                f"{car_index + 1:02d}"
            )
            traci.vehicle.add(
                vehicle_id,
                f"initial_east_queue_{movement}",
                typeID="DEFAULT_VEHTYPE",
                depart="now",
                departLane=str(lane_index),
                departPos="base",
                departSpeed="0",
            )
            # TraCI normally queues new vehicles for insertion on the next
            # simulation step. moveTo force-inserts the loaded vehicle now,
            # allowing the pre-backend queue to exist while SUMO remains at
            # its current time (0 on startup).
            traci.vehicle.moveTo(
                vehicle_id,
                f"{INITIAL_QUEUE_EDGE}_{lane_index}",
                position,
            )
            traci.vehicle.setSpeed(vehicle_id, 0)
            vehicle_ids.append(vehicle_id)

    present = set(traci.vehicle.getIDList())
    missing = sorted(set(vehicle_ids) - present)
    if missing:
        raise RuntimeError(f"failed to force-insert staged vehicles: {missing}")

    lane_counts = {
        lane_index: sum(
            traci.vehicle.getLaneID(vehicle_id) == f"{INITIAL_QUEUE_EDGE}_{lane_index}"
            for vehicle_id in vehicle_ids
        )
        for lane_index in range(INITIAL_QUEUE_LANES)
    }
    if any(count != INITIAL_QUEUE_PER_LANE for count in lane_counts.values()):
        raise RuntimeError(f"staged east queue has incorrect lane distribution: {lane_counts}")

    current_time = traci.simulation.getTime()
    if current_time != staged_time:
        raise RuntimeError(
            f"staging the initial queue advanced SUMO time: {staged_time} -> {current_time}"
        )

    print(
        f"[SUMO] preloaded stationary east-to-west queue for batch {batch_number}: "
        f"{INITIAL_QUEUE_LANES} lanes x {INITIAL_QUEUE_PER_LANE} vehicles "
        f"({len(vehicle_ids)} total); held at simulation time {staged_time:.1f}s"
    )
    return vehicle_ids


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


def _ensure_emergency_vehicle_type():
    if EMERGENCY_VEHICLE_TYPE not in traci.vehicletype.getIDList():
        traci.vehicletype.copy("DEFAULT_VEHTYPE", EMERGENCY_VEHICLE_TYPE)
    traci.vehicletype.setColor(EMERGENCY_VEHICLE_TYPE, EMERGENCY_COLOR)


def _sumo_emergency_id(backend_id):
    safe_id = re.sub(r"[^A-Za-z0-9_.-]", "_", backend_id.strip())
    return f"emergency_{safe_id}"


def _spawn_emergency_vehicle(direction, backend_vehicle_id, seen_ids):
    """Insert one red SUMO vehicle per backend tracking ID."""
    if backend_vehicle_id in seen_ids:
        return None
    seen_ids.add(backend_vehicle_id)
    vehicle_id = _sumo_emergency_id(backend_vehicle_id)
    if vehicle_id in traci.vehicle.getIDList():
        print(
            f"[SUMO] emergency vehicle already present: backend_id={backend_vehicle_id}, "
            f"sumo_id={vehicle_id}; spawn skipped"
        )
        return vehicle_id
    _ensure_emergency_vehicle_type()
    route_id = f"backend_route_{direction.lower()}_straight"
    traci.vehicle.add(
        vehicle_id,
        route_id,
        typeID=EMERGENCY_VEHICLE_TYPE,
        depart="now",
        departLane="best",
        departPos="base",
        departSpeed="0",
    )
    print(
        f"[SUMO] emergency vehicle spawn requested: direction={direction}, "
        f"backend_id={backend_vehicle_id}, sumo_id={vehicle_id}, route=straight"
    )
    return vehicle_id


def _begin_emergency_priority(direction, controller, sim_time):
    """Yield safely to an approach, remembering where the regular cycle paused."""
    if traci.trafficlight.getProgram(TLS_ID) != "0":
        traci.trafficlight.setProgram(TLS_ID, "0")
    phase = traci.trafficlight.getPhase(TLS_ID)
    target_phase = GREEN_PHASES[direction]

    if phase in PHASE_DIRECTIONS:
        current_direction = PHASE_DIRECTIONS[phase]
        if current_direction == direction:
            resume_phase = (phase + 2) % 8
            traci.trafficlight.setRedYellowGreenState(
                TLS_ID, TLS_PHASE_STATES[target_phase]
            )
            controller.update(
                state="PRIORITY_GREEN", resume_phase=resume_phase,
                direction=direction, deadline=None,
            )
            print(
                f"[SUMO] event_state=ACTIVE; emergency priority already has green: {direction}; "
                f"TLS phase={target_phase}, "
                f"state={traci.trafficlight.getRedYellowGreenState(TLS_ID)}; "
                f"will resume at phase {resume_phase} after CLEARED"
            )
            return

        resume_phase = phase
        yellow_phase = phase + 1
    else:
        # If interrupted during yellow, finish a full existing clearance and
        # resume with the green that normally follows that yellow.
        yellow_phase = phase if phase % 2 else (phase + 1) % 8
        resume_phase = (yellow_phase + 1) % 8

    yellow_duration = EMERGENCY_YELLOW_SECONDS
    traci.trafficlight.setRedYellowGreenState(
        TLS_ID, TLS_PHASE_STATES[yellow_phase]
    )
    controller.update(
        state="CLEARING_TO_PRIORITY", resume_phase=resume_phase,
        direction=direction, deadline=sim_time + yellow_duration,
    )
    print(
        f"[SUMO] event_state=ACTIVE; emergency {direction} requested during phase {phase}; "
        f"holding existing yellow clearance phase {yellow_phase} for "
        f"{yellow_duration:g}s before priority green"
    )


def _begin_emergency_resume(controller, sim_time):
    direction = controller["direction"]
    yellow_phase = GREEN_PHASES[direction] + 1
    yellow_duration = EMERGENCY_YELLOW_SECONDS
    traci.trafficlight.setRedYellowGreenState(
        TLS_ID, TLS_PHASE_STATES[yellow_phase]
    )
    controller["state"] = "CLEARING_TO_RESUME"
    controller["deadline"] = sim_time + yellow_duration
    print(
        f"[SUMO] event_state=CLEARED; emergency vehicle cleared; applying {direction} yellow "
        f"clearance for {yellow_duration:g}s before resuming phase "
        f"{controller['resume_phase']}"
    )


def _advance_emergency_controller(controller, sim_time, active_green_times):
    """Advance emergency override timers; return True while override owns TLS."""
    state = controller["state"]
    if state == "IDLE":
        return False
    if state == "CLEARING_TO_PRIORITY" and sim_time >= controller["deadline"]:
        direction = controller["direction"]
        phase = GREEN_PHASES[direction]
        traci.trafficlight.setRedYellowGreenState(TLS_ID, TLS_PHASE_STATES[phase])
        controller["state"] = "PRIORITY_GREEN"
        controller["deadline"] = None
        print(
            f"[SUMO] event_state=ACTIVE; applied emergency priority phase {phase} ({direction} green); "
            f"state={TLS_PHASE_STATES[phase]}; "
            "holding until backend event_state=CLEARED"
        )
    elif state == "CLEARING_TO_RESUME" and sim_time >= controller["deadline"]:
        resume_phase = controller["resume_phase"]
        traci.trafficlight.setProgram(TLS_ID, "0")
        traci.trafficlight.setPhase(TLS_ID, resume_phase)
        direction = PHASE_DIRECTIONS[resume_phase]
        if active_green_times:
            duration = active_green_times[direction]
            traci.trafficlight.setPhaseDuration(TLS_ID, duration)
            print(
                f"[SUMO] resumed regular cycle at phase {resume_phase} "
                f"({direction} green) for {duration:g}s using latest backend timing; "
                "event_state=CLEARED; emergency return complete"
            )
        else:
            print(
                f"[SUMO] resumed existing SUMO cycle at phase {resume_phase} "
                f"({direction} green); event_state=CLEARED; emergency return complete"
            )
        controller.update(
            state="IDLE", direction=None, vehicle_id=None,
            resume_phase=None, deadline=None,
        )
    return controller["state"] != "IDLE"


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
    print("=== Backend to SUMO congestion demo ===")
    print(f"[SUMO] config: {SUMO_CONFIG}")
    print(f"[backend] control: GET {SIGNAL_CONTROL_URL}")

    # SUMO-GUI needs to be started for the TraCI session to initialize reliably;
    # the bridge still controls advancement through explicit simulationStep calls.
    traci.start([
        SUMO_PATH,
        "-c", SUMO_CONFIG,
        "--start",
        "--log", str(SUMO_LOG_FILE),
        "--error-log", str(SUMO_ERROR_LOG_FILE),
    ], verbose=True)
    print("[SUMO] TraCI connected")
    traci.trafficlight.setProgram(TLS_ID, "0")
    traci.trafficlight.setPhase(TLS_ID, GREEN_PHASES["east"])
    print("[SUMO] initial signal set to phase 0 (east green) before backend data")
    for direction, movements in MOVEMENT_ROUTE_EDGES.items():
        for movement, edges in movements.items():
            traci.route.add(f"backend_route_{direction}_{movement}", list(edges))
    for movement, edges in MOVEMENT_ROUTE_EDGES["east"].items():
        traci.route.add(f"initial_east_queue_{movement}", list(edges))
    cycle_number = 1
    initial_queue_ids = _stage_initial_east_queue(cycle_number)
    initial_queue_released = False
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
    waiting_logged = False
    armed_for_active = False
    stale_active_logged = False
    waiting_seen_during_run = False
    seen_emergency_ids = set()
    active_emergency_sumo_id = None
    emergency_spawn_confirmed = False
    emergency_vehicle_finished = False
    current_backend_event_state = "WAITING"
    last_event_log_signature = None
    last_tls_snapshot = (
        traci.trafficlight.getPhase(TLS_ID),
        traci.trafficlight.getRedYellowGreenState(TLS_ID),
    )
    print(
        f"[SUMO] initial TLS state: event_state={current_backend_event_state}, "
        f"phase={last_tls_snapshot[0]} ({PHASE_LABELS.get(last_tls_snapshot[0], 'unknown')}), "
        f"state={last_tls_snapshot[1]}"
    )
    emergency_controller = {
        "state": "IDLE", "direction": None, "vehicle_id": None,
        "resume_phase": None, "deadline": None,
    }

    try:
        while True:
            state, snapshot = get_backend_snapshot(session)
            sim_time = traci.simulation.getTime()

            state_changed = state != last_status
            if state_changed:
                print(f"[backend] connection/status: {state}")
                last_status = state

            if state == "WAITING" and state_changed:
                current_backend_event_state = "WAITING"
                waiting_event_log = json.dumps(
                    {
                        "event_state": "WAITING", "mode": None,
                        "priority_direction": None, "emergency_vehicle_id": None,
                        "active_events": [], "green_times": None,
                    },
                    sort_keys=True,
                )
                if waiting_event_log != last_event_log_signature:
                    print(
                        "[backend] event/control fields changed:",
                        waiting_event_log,
                    )
                    last_event_log_signature = waiting_event_log
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
                    waiting_seen_during_run = True
                    print("[SUMO] one-shot run already started; finishing with last received control")

            if state in ("ACTIVE", "ACTIVE_EVENT"):
                counts = snapshot["counts"]
                movement_counts = snapshot["movement_counts"]
                result = snapshot["result"]
                event_state = snapshot.get("event_state") or "NONE"
                current_backend_event_state = event_state
                event_log_payload = {
                    "event_state": event_state,
                    "mode": result.get("mode"),
                    "priority_direction": result.get("priority_direction"),
                    "emergency_vehicle_id": result.get("emergency_vehicle_id"),
                    "active_events": result.get("active_events", []),
                    "green_times": result.get("green_times"),
                }
                event_log_signature = json.dumps(
                    event_log_payload, sort_keys=True, ensure_ascii=False
                )
                if event_log_signature != last_event_log_signature:
                    print("[backend] event/control fields changed:", event_log_signature)
                    last_event_log_signature = event_log_signature
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
                    if movement_counts:
                        print(
                            "[backend] direction movement counts:",
                            json.dumps(movement_counts, ensure_ascii=False),
                        )
                    last_data_signature = data_signature
                wall_now = time.monotonic()
                valid_counts = (
                    not snapshot.get("counts_error")
                    and len(counts or {}) == len(APPROACH_EDGES)
                    and len(movement_counts or {}) == len(APPROACH_EDGES)
                )
                event_state = snapshot.get("event_state", "")
                priority_direction = snapshot.get("priority_direction")
                emergency_vehicle_id = snapshot.get("emergency_vehicle_id")
                valid_emergency_event = (
                    event_state == "ACTIVE"
                    and priority_direction in APPROACH_EDGES
                    and isinstance(emergency_vehicle_id, str)
                    and bool(emergency_vehicle_id.strip())
                )

                if wall_now - last_backend_log_wall >= 5:
                    print(
                        f"[backend] GET /api/control/signal?intersection_id={INTERSECTION_ID} reachable; "
                        f"current mode={result.get('mode', 'MISSING')}"
                    )
                    last_backend_log_wall = wall_now

                if (
                    (armed_for_active or fresh_payload)
                    and not simulation_started
                    and (
                        (valid_counts and sum(counts.values()) > 0)
                        or valid_emergency_event
                    )
                ):
                    pending_spawn_plan = (
                        _build_spawn_plan(counts, movement_counts, cycle_number)
                        if valid_counts and sum(counts.values()) > 0 else []
                    )
                    simulation_ready = True
                    simulation_started = True
                    armed_for_active = False
                    stale_active_logged = False
                    waiting_logged = False
                    trigger = "new/changed ACTIVE backend response"
                    print(
                        f"[SUMO] {trigger}; creating one finite batch of "
                        f"{sum(counts.values()) if valid_counts else 0} ordinary "
                        "vehicles from backend counts"
                    )
                    if valid_counts:
                        print("[SUMO] one-shot vehicle counts:", json.dumps(counts, ensure_ascii=False))
                        print(
                            "[SUMO] backend movement counts:",
                            json.dumps(movement_counts, ensure_ascii=False),
                        )
                    if valid_emergency_event:
                        print(
                            f"[SUMO] emergency event also starts the run despite ordinary "
                            f"vehicle count={sum(counts.values()) if valid_counts else 'unavailable'}"
                        )
                elif (
                    (armed_for_active or fresh_payload)
                    and valid_counts
                    and sum(counts.values()) == 0
                    and not stale_active_logged
                ):
                    print("[SUMO] all backend direction counts are zero; waiting without starting")
                    stale_active_logged = True
                elif snapshot.get("counts_error") and not stale_active_logged:
                    print(
                        "[SUMO] backend data rejected; waiting for valid direction totals and "
                        f"movement counts: {snapshot['counts_error']}"
                    )
                    stale_active_logged = True
                elif not armed_for_active and not simulation_ready and not stale_active_logged:
                    print(
                        "[SUMO] cached ACTIVE response ignored; waiting for a changed "
                        "backend payload or WAITING -> ACTIVE transition"
                    )
                    stale_active_logged = True

                if valid_emergency_event and simulation_ready:
                    if emergency_controller["state"] == "IDLE":
                        emergency_controller["vehicle_id"] = emergency_vehicle_id
                        emergency_controller["direction"] = priority_direction
                        active_emergency_sumo_id = _spawn_emergency_vehicle(
                            priority_direction, emergency_vehicle_id, seen_emergency_ids
                        )
                        emergency_spawn_confirmed = (
                            active_emergency_sumo_id in traci.vehicle.getIDList()
                            if active_emergency_sumo_id else False
                        )
                        emergency_vehicle_finished = False
                        _begin_emergency_priority(
                            priority_direction, emergency_controller, sim_time
                        )
                    elif emergency_controller["vehicle_id"] != emergency_vehicle_id:
                        print(
                            f"[SUMO] concurrent emergency event {emergency_vehicle_id} "
                            f"ignored while {emergency_controller['vehicle_id']} is active"
                        )
                elif event_state == "CLEARED":
                    restored_plan = snapshot.get("green_times")
                    if restored_plan is not None and restored_plan != active_green_times:
                        active_green_times = restored_plan
                        print(
                            "[SUMO] backend restored camera 1 green times:",
                            json.dumps(restored_plan, ensure_ascii=False),
                        )
                    if emergency_controller["state"] not in ("IDLE", "CLEARING_TO_RESUME"):
                        _begin_emergency_resume(emergency_controller, sim_time)

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
                            f"{plan_source}; E={active_green_times['east']:g}s, "
                            f"N={active_green_times['north']:g}s, "
                            f"S={active_green_times['south']:g}s, "
                            f"W={active_green_times['west']:g}s"
                        )
                        if emergency_controller["state"] != "IDLE":
                            print(
                                f"[SUMO] restored camera 1 timing stored; emergency "
                                f"clearance remains active in state {emergency_controller['state']}"
                            )
                        elif traci.trafficlight.getProgram(TLS_ID) != "0":
                            traci.trafficlight.setProgram(TLS_ID, "0")
                        if emergency_controller["state"] == "IDLE" and simulation_ready:
                            run_control_latched = True
                            waiting_logged = False
                            print(
                                f"[SUMO] latched {plan_source} for this one-shot batch; "
                                "applying at green phase starts and holding until completion"
                            )
                        elif emergency_controller["state"] == "IDLE":
                            print(
                                f"[SUMO] {plan_source} is present in the baseline response; "
                                "waiting for a subsequent backend update"
                            )
                    elif mode == "EMERGENCY_VEHICLE_PRIORITY" and event_state != "ACTIVE":
                        if simulation_ready:
                            active_green_times = None
                            waiting_logged = False
                            traci.trafficlight.setProgram(TLS_ID, "legacy_events")
                            traci.trafficlight.setPhase(TLS_ID, 0)
                            print("[SUMO] preserved legacy emergency mapping: legacy_events phase 0")
                    elif mode == "EMERGENCY_VEHICLE_PRIORITY" and event_state == "ACTIVE":
                        if simulation_ready:
                            print(
                                "[SUMO] emergency event fields accepted; priority is controlled "
                                "by the event state machine"
                            )
                        else:
                            print(
                                "[SUMO] cached emergency ACTIVE response observed; waiting for "
                                "WAITING -> ACTIVE or a changed backend response before starting"
                            )
                    elif "ACCIDENT" in mode:
                        if simulation_ready:
                            active_green_times = None
                            waiting_logged = False
                            traci.trafficlight.setProgram(TLS_ID, "legacy_events")
                            traci.trafficlight.setPhase(TLS_ID, 2)
                            print("[SUMO] preserved legacy accident mapping: legacy_events phase 2")
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

            # Keep the staged queue and SUMO time fixed until the backend
            # publishes a supported control result.
            if not simulation_ready:
                if not waiting_logged:
                    print("[SUMO] paused at start; waiting for a valid backend control result")
                    waiting_logged = True
                time.sleep(POLL_SECONDS)
                continue

            if not initial_queue_released:
                for vehicle_id in initial_queue_ids:
                    traci.vehicle.setSpeed(vehicle_id, -1)
                initial_queue_released = True
                print(
                    f"[SUMO] backend activated; released the staged "
                    f"{len(initial_queue_ids)} east-to-west vehicles"
                )

            spawned_this_step = {direction: 0 for direction in APPROACH_EDGES}
            spawned_movements = {
                direction: {movement: 0 for movement in MOVEMENT_ROUTE_EDGES[direction]}
                for direction in APPROACH_EDGES
            }
            while pending_spawn_plan and pending_spawn_plan[0][0] <= sim_time:
                _, direction, movement, vehicle_id = pending_spawn_plan.pop(0)
                traci.vehicle.add(
                    vehicle_id,
                    f"backend_route_{direction}_{movement}",
                    typeID="DEFAULT_VEHTYPE",
                    depart="now",
                    departLane="best",
                    departPos="base",
                    departSpeed="0",
                )
                spawned_this_step[direction] += 1
                spawned_movements[direction][movement] += 1
            spawned_now = {d: n for d, n in spawned_this_step.items() if n}
            if spawned_now:
                print("[SUMO] released backend vehicles:", json.dumps(spawned_now))
                print(
                    "[SUMO] released by movement:",
                    json.dumps(
                        {d: counts for d, counts in spawned_movements.items() if any(counts.values())},
                        ensure_ascii=False,
                    ),
                )

            emergency_override_active = _advance_emergency_controller(
                emergency_controller, sim_time, active_green_times
            )
            phase = traci.trafficlight.getPhase(TLS_ID)
            if (
                not emergency_override_active
                and traci.trafficlight.getProgram(TLS_ID) == "0"
                and phase in PHASE_DIRECTIONS
                and phase != last_phase
                and active_green_times
            ):
                direction = PHASE_DIRECTIONS[phase]
                duration = active_green_times[direction]
                baseline_queues = _queue_snapshot()
                traci.trafficlight.setPhaseDuration(TLS_ID, duration)
                comparison_label = f"backend control applied to {direction}; baseline queue captured"
                window_start_queues = baseline_queues
                window_passes = {direction: 0 for direction in APPROACH_EDGES}
                window_start_time = sim_time
                print(
                    f"[SUMO] applied phase {phase} ({direction} green) for {duration:g}s; "
                    "yellow clearance phases remain unchanged"
                )

            traci.simulationStep()
            current_vehicle_ids = set(traci.vehicle.getIDList())
            if (
                active_emergency_sumo_id
                and active_emergency_sumo_id in current_vehicle_ids
                and not emergency_spawn_confirmed
            ):
                emergency_spawn_confirmed = True
                print(
                    f"[SUMO] emergency vehicle spawned in simulation at "
                    f"t={traci.simulation.getTime():.1f}s: {active_emergency_sumo_id}"
                )
            current_roads = {
                vehicle_id: traci.vehicle.getRoadID(vehicle_id)
                for vehicle_id in current_vehicle_ids
            }
            step_passes = _incoming_to_green_passes(previous_roads, current_roads)
            for direction in window_passes:
                window_passes[direction] += step_passes[direction]
            previous_roads = current_roads

            now = traci.simulation.getTime()
            arrived_ids = set(traci.simulation.getArrivedIDList())
            for arrived_id in arrived_ids:
                if arrived_id.startswith("emergency_"):
                    print(
                        f"[SUMO] emergency vehicle passed through and reached its "
                        f"route destination at t={now:.1f}s: {arrived_id}"
                    )
                    if arrived_id == active_emergency_sumo_id:
                        emergency_vehicle_finished = True
            if (
                active_emergency_sumo_id
                and emergency_spawn_confirmed
                and not emergency_vehicle_finished
                and active_emergency_sumo_id not in current_vehicle_ids
                and active_emergency_sumo_id not in arrived_ids
            ):
                emergency_vehicle_finished = True
                print(
                    f"[SUMO] emergency vehicle disappeared before route arrival was reported "
                    f"at t={now:.1f}s: {active_emergency_sumo_id}"
                )
            current_tls_snapshot = (
                traci.trafficlight.getPhase(TLS_ID),
                traci.trafficlight.getRedYellowGreenState(TLS_ID),
            )
            if current_tls_snapshot != last_tls_snapshot:
                phase, signal_state = current_tls_snapshot
                print(
                    f"[SUMO] TLS changed at t={now:.1f}s; "
                    f"event_state={current_backend_event_state}; "
                    f"phase={phase} ({PHASE_LABELS.get(phase, 'online/custom')}), "
                    f"state={signal_state}"
                )
                last_tls_snapshot = current_tls_snapshot
            if now - window_start_time >= METRICS_INTERVAL:
                window_start_queues = _print_metrics(
                    window_start_queues, window_passes, int(now - window_start_time), comparison_label
                )
                window_passes = {direction: 0 for direction in APPROACH_EDGES}
                window_start_time = now
                if active_green_times:
                    comparison_label = (
                        "backend timing active; "
                        f"E={active_green_times['east']:g}s, "
                        f"N={active_green_times['north']:g}s, "
                        f"S={active_green_times['south']:g}s, "
                        f"W={active_green_times['west']:g}s"
                    )
                else:
                    comparison_label = "existing SUMO program"

            if (
                not pending_spawn_plan
                and traci.simulation.getMinExpectedNumber() <= 0
                and emergency_controller["state"] == "IDLE"
            ):
                simulation_ready = False
                simulation_started = False
                run_control_latched = False
                active_green_times = None
                emergency_controller.update(
                    state="IDLE", direction=None, vehicle_id=None,
                    resume_phase=None, deadline=None,
                )
                pending_spawn_plan = []
                last_result_signature = None
                last_data_signature = None
                last_phase = None
                waiting_logged = False
                stale_active_logged = False
                armed_for_active = waiting_seen_during_run
                waiting_seen_during_run = False
                cycle_number += 1
                initial_queue_ids = _stage_initial_east_queue(cycle_number)
                initial_queue_released = False
                previous_roads = {
                    vehicle_id: traci.vehicle.getRoadID(vehicle_id)
                    for vehicle_id in initial_queue_ids
                }
                window_start_time = traci.simulation.getTime()
                window_start_queues = _queue_snapshot()
                window_passes = {direction: 0 for direction in APPROACH_EDGES}
                comparison_label = "staged east-to-west queue; awaiting next backend batch"
                print(
                    f"[SUMO] one-shot backend batch completed at simulation time {now:.1f}s; "
                    f"waiting for the next backend batch (cycle {cycle_number - 1} complete)"
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
