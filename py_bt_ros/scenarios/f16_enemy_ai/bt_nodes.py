import time

from modules.base_bt_nodes import (
    BTNodeList,
    Fallback,
    Node,
    Parallel,
    ReactiveFallback,
    ReactiveSequence,
    Sequence,
    Status,
    SyncCondition,
)
from modules.utils import config
from scenarios.f16_enemy_ai.flight_control import (
    altitude_m,
    build_guidance_command,
    format_command,
    limited_heading_toward,
    throttle_for_spacing,
)
from scenarios.f16_enemy_ai.geometry import (
    behind_aspect_error_deg,
    bearing_deg,
    clamp,
    course_deg_from_state,
    distance_m,
    forward_vector_xy_from_state,
    lead_intercept_point,
    pitch_deg,
    point_from_state,
    rear_anchor_solution,
    wrap_angle_deg,
)

_CONFIG = config or {}
_AI_CFG = _CONFIG.get('f16_ai', {})
_CONTROLLER_CFG = _AI_CFG.get('controller', {})
_RECOVERY_CFG = _AI_CFG.get('recovery', {})
_TAKEOFF_CFG = _AI_CFG.get('takeoff', {})
_CLIMB_CFG = _AI_CFG.get('climb', {})
_INTERCEPT_CFG = _AI_CFG.get('intercept', {})
_GET_BEHIND_CFG = _AI_CFG.get('get_behind', {})
_FOLLOW_CFG = _AI_CFG.get('follow', {})
_REACQUIRE_CFG = _AI_CFG.get('reacquire', {})
_LOGGING_CFG = _AI_CFG.get('logging', {})
_DEBUG_CFG = _AI_CFG.get('debug', {})
_FORCE_TAKEOFF_CFG = _DEBUG_CFG.get('force_takeoff', {})
_COORDINATE_LOGGED = False

CUSTOM_ACTION_NODES = [
    'Takeoff',
    'ClimbToSafeAltitude',
    'InterceptTarget',
    'GetBehindTarget',
    'FollowTarget',
    'ReacquireTarget',
    'Recover',
    'Reposition',
]

CUSTOM_CONDITION_NODES = [
    'IsAirborne',
    'IsStalling',
    'NeedsTakeoff',
    'NeedsClimbToSafeAltitude',
    'TargetFar',
    'TargetInRange',
    'IsBehindTarget',
    'TargetLost',
]

for node_name in CUSTOM_ACTION_NODES:
    if node_name not in BTNodeList.ACTION_NODES:
        BTNodeList.ACTION_NODES.append(node_name)

for node_name in CUSTOM_CONDITION_NODES:
    if node_name not in BTNodeList.CONDITION_NODES:
        BTNodeList.CONDITION_NODES.append(node_name)


def _state_is_valid(state):
    return state is not None and getattr(state, 'valid', False)


def _get_states(agent, blackboard):
    return blackboard.get('ego_state'), blackboard.get('enemy_state')


def _debug_log(message):
    if _LOGGING_CFG.get('enabled', True):
        print(f"[F16BT] {message}")


def _condition_log(blackboard, name, result, reason):
    trace = blackboard.setdefault('bt_tick_condition_trace', [])
    trace.append(f"{name}={result}:{reason}")
    if _DEBUG_CFG.get('enabled', False):
        _debug_log(f"Condition {name} -> {result} reason={reason}")


def _record_branch_attempt(blackboard, node_name, branch_name, command):
    attempts = blackboard.setdefault('bt_tick_branch_attempts', [])
    attempts.append(
        f"{node_name}:{branch_name}:thr={command[0]:.2f},ele={command[1]:.2f},ail={command[2]:.2f},rud={command[3]:.2f}"
    )


def _force_takeoff_active(blackboard):
    if not _FORCE_TAKEOFF_CFG.get('enabled', False):
        blackboard['f16_force_takeoff_active'] = False
        return False

    start_time = blackboard.get('f16_force_takeoff_start_s')
    if start_time is None:
        start_time = time.monotonic()
        blackboard['f16_force_takeoff_start_s'] = start_time

    active = (time.monotonic() - float(start_time)) <= float(_FORCE_TAKEOFF_CFG.get('duration_s', 8.0))
    blackboard['f16_force_takeoff_active'] = active
    return active


def _log_coordinate_convention_once():
    global _COORDINATE_LOGGED
    if _COORDINATE_LOGGED:
        return
    _COORDINATE_LOGGED = True
    _debug_log(
        "CoordCheck world_axes=(x,y,z-up) bearing=atan2(dy,dx) "
        "heading_error=wrap(target_bearing-ego_yaw) "
        "cmd_mapping=(aileron->roll,elevator->pitch,rudder->yaw)"
    )


def _snapshot_target_track(blackboard, target):
    if not _state_is_valid(target):
        return None

    track = {
        'position': point_from_state(target),
        'velocity': {
            'x': float(target.velocity_mps.x),
            'y': float(target.velocity_mps.y),
            'z': float(target.velocity_mps.z),
        },
        'course_deg': float(course_deg_from_state(target)),
        'airspeed_mps': float(target.airspeed_mps),
        'yaw_deg': float(target.yaw_deg),
        'pitch_deg': float(target.pitch_deg),
        'roll_deg': float(target.roll_deg),
        'timestamp_s': float(getattr(target, 'timestamp_s', 0.0) or time.monotonic()),
    }
    blackboard['f16_last_target_track'] = track
    return track


def _get_last_target_track(blackboard):
    return blackboard.get('f16_last_target_track')


def _track_age_s(track):
    if not track:
        return float('inf')
    return max(0.0, time.monotonic() - float(track.get('timestamp_s', 0.0)))


def _track_is_recent(blackboard):
    track = _get_last_target_track(blackboard)
    if not track:
        return False
    return _track_age_s(track) <= float(_REACQUIRE_CFG.get('max_memory_age_s', 8.0))


def _takeoff_exit_altitude_m():
    return float(
        _TAKEOFF_CFG.get(
            'takeoff_exit_altitude_m',
            max(float(_AI_CFG.get('airborne_altitude_m', 20.0)), 80.0),
        )
    )


def _takeoff_exit_speed_mps():
    return float(
        _TAKEOFF_CFG.get(
            'takeoff_exit_speed_mps',
            max(float(_TAKEOFF_CFG.get('rotate_speed_mps', 82.0)), 90.0),
        )
    )


def _takeoff_exit_reached(ego):
    return (
        (not ego.on_ground)
        and altitude_m(ego) >= _takeoff_exit_altitude_m()
        and ego.airspeed_mps >= _takeoff_exit_speed_mps()
    )


def _takeoff_command():
    return (
        float(_TAKEOFF_CFG.get('takeoff_throttle_cmd', 0.90)),
        float(_TAKEOFF_CFG.get('takeoff_pitch_cmd', 0.15)),
        float(_TAKEOFF_CFG.get('takeoff_roll_cmd', 0.0)),
        float(_TAKEOFF_CFG.get('takeoff_yaw_cmd', 0.0)),
    )


def _takeoff_phase_command(ego):
    rotate_speed_mps = float(_TAKEOFF_CFG.get('rotate_speed_mps', 82.0))
    airborne_altitude_m = float(_AI_CFG.get('airborne_altitude_m', 20.0))
    current_altitude = altitude_m(ego)
    current_speed = float(ego.airspeed_mps)

    if ego.on_ground and current_speed < (rotate_speed_mps * 0.92):
        return (
            "ground_roll",
            (
                float(_TAKEOFF_CFG.get('ground_roll_throttle_cmd', 1.0)),
                float(_TAKEOFF_CFG.get('ground_roll_pitch_cmd', 0.0)),
                float(_TAKEOFF_CFG.get('ground_roll_roll_cmd', 0.0)),
                float(_TAKEOFF_CFG.get('ground_roll_yaw_cmd', 0.0)),
            ),
        )

    if ego.on_ground or current_altitude < airborne_altitude_m:
        return (
            "rotation",
            (
                float(_TAKEOFF_CFG.get('rotation_throttle_cmd', 1.0)),
                float(_TAKEOFF_CFG.get('rotation_pitch_cmd', 0.22)),
                float(_TAKEOFF_CFG.get('rotation_roll_cmd', 0.0)),
                float(_TAKEOFF_CFG.get('rotation_yaw_cmd', 0.0)),
            ),
        )

    return (
        "initial_climb",
        (
            float(_TAKEOFF_CFG.get('initial_climb_throttle_cmd', 1.0)),
            float(_TAKEOFF_CFG.get('initial_climb_pitch_cmd', 0.18)),
            float(_TAKEOFF_CFG.get('initial_climb_roll_cmd', 0.0)),
            float(_TAKEOFF_CFG.get('initial_climb_yaw_cmd', 0.0)),
        ),
    )


def _forced_takeoff_phase_command(ego):
    rotate_speed_mps = float(_TAKEOFF_CFG.get('rotate_speed_mps', 82.0))
    airborne_altitude_m = float(_AI_CFG.get('airborne_altitude_m', 20.0))
    current_altitude = altitude_m(ego)
    current_speed = float(ego.airspeed_mps)

    if ego.on_ground and current_speed < (rotate_speed_mps * 0.92):
        return (
            "forced_ground_roll",
            (
                float(_FORCE_TAKEOFF_CFG.get('ground_roll_throttle_cmd', 1.0)),
                float(_FORCE_TAKEOFF_CFG.get('ground_roll_pitch_cmd', 0.0)),
                0.0,
                0.0,
            ),
        )

    if ego.on_ground or current_altitude < airborne_altitude_m:
        return (
            "forced_rotation",
            (
                float(_FORCE_TAKEOFF_CFG.get('rotation_throttle_cmd', 1.0)),
                float(_FORCE_TAKEOFF_CFG.get('rotation_pitch_cmd', 0.30)),
                0.0,
                0.0,
            ),
        )

    return (
        "forced_initial_climb",
        (
            float(_FORCE_TAKEOFF_CFG.get('initial_climb_throttle_cmd', 1.0)),
            float(_FORCE_TAKEOFF_CFG.get('initial_climb_pitch_cmd', 0.20)),
            0.0,
            0.0,
        ),
    )


def _far_range_m():
    return float(_AI_CFG.get('far_range_m', _AI_CFG.get('target_far_distance_m', 6000.0)))


def _in_range_m():
    return float(_AI_CFG.get('in_range_m', _AI_CFG.get('target_in_range_m', 2500.0)))


def _follow_distance_m():
    return float(_AI_CFG.get('follow_distance_m', _FOLLOW_CFG.get('distance_m', 550.0)))


def _rear_aspect_angle_deg():
    return float(_AI_CFG.get('rear_aspect_angle_deg', _FOLLOW_CFG.get('max_aspect_error_deg', 35.0)))


def _is_airborne_now(ego):
    airborne_altitude = float(_AI_CFG.get('airborne_altitude_m', 20.0))
    return (not ego.on_ground) and (altitude_m(ego) >= airborne_altitude)


def _needs_takeoff(ego, blackboard=None):
    if _takeoff_exit_reached(ego):
        if isinstance(blackboard, dict):
            blackboard['f16_takeoff_committed'] = False
        return False

    if isinstance(blackboard, dict) and blackboard.get('f16_takeoff_committed', False):
        return True

    return not _is_airborne_now(ego)


def _needs_safe_altitude(ego):
    safe_altitude = float(_AI_CFG.get('safe_altitude_m', 1500.0))
    return _is_airborne_now(ego) and altitude_m(ego) < safe_altitude


def _update_target_metrics(blackboard, ego, target):
    if not (_state_is_valid(ego) and _state_is_valid(target)):
        return None

    _log_coordinate_convention_once()
    _snapshot_target_track(blackboard, target)

    ego_point = point_from_state(ego)
    target_point = point_from_state(target)
    anchor_distance_m = _follow_distance_m()
    anchor_lookahead_s = float(_FOLLOW_CFG.get('lookahead_s', 0.8))
    anchor_solution = rear_anchor_solution(
        target,
        distance_back_m=anchor_distance_m,
        lookahead_s=anchor_lookahead_s,
        altitude_offset_m=float(_FOLLOW_CFG.get('altitude_offset_m', 0.0)),
        min_velocity_mps=float(_FOLLOW_CFG.get('min_target_forward_speed_mps', 25.0)),
    )
    behind_anchor = anchor_solution['anchor']
    target_forward = forward_vector_xy_from_state(
        target,
        min_velocity_mps=float(_FOLLOW_CFG.get('min_target_forward_speed_mps', 25.0)),
        return_meta=True,
    )

    metrics = {
        'distance_m': distance_m(ego_point, target_point),
        'bearing_deg': bearing_deg(ego_point, target_point),
        'pitch_to_target_deg': pitch_deg(ego_point, target_point),
        'altitude_error_m': target.position_m.z - ego.position_m.z,
        'relative_x_m': target.position_m.x - ego.position_m.x,
        'relative_y_m': target.position_m.y - ego.position_m.y,
        'relative_z_m': target.position_m.z - ego.position_m.z,
        'target_position_x_m': float(target.position_m.x),
        'target_position_y_m': float(target.position_m.y),
        'target_position_z_m': float(target.position_m.z),
        'target_velocity_x_mps': float(target.velocity_mps.x),
        'target_velocity_y_mps': float(target.velocity_mps.y),
        'target_velocity_z_mps': float(target.velocity_mps.z),
        'target_yaw_deg': float(target.yaw_deg),
        'target_pitch_deg': float(target.pitch_deg),
        'target_roll_deg': float(target.roll_deg),
        'target_course_deg': course_deg_from_state(target),
        'target_forward_source': str(target_forward['source']),
        'target_forward_x': float(target_forward['forward_x']),
        'target_forward_y': float(target_forward['forward_y']),
        'behind_aspect_error_deg': behind_aspect_error_deg(
            ego,
            target,
            min_velocity_mps=float(_FOLLOW_CFG.get('min_target_forward_speed_mps', 25.0)),
        ),
        'behind_anchor_distance_m': distance_m(ego_point, behind_anchor),
        'rear_anchor_x_m': float(behind_anchor['x']),
        'rear_anchor_y_m': float(behind_anchor['y']),
        'rear_anchor_z_m': float(behind_anchor['z']),
        'rear_anchor_source': str(anchor_solution['source']),
        'rear_anchor_course_deg': float(anchor_solution['course_deg']),
    }
    metrics['heading_error_deg'] = wrap_angle_deg(metrics['bearing_deg'] - ego.yaw_deg)
    blackboard['f16_target_metrics'] = metrics
    return metrics


def _format_target_metrics(metrics, include_anchor=False):
    if not metrics:
        return ""

    parts = [
        (
            f"target_pos=({metrics['target_position_x_m']:.1f},"
            f"{metrics['target_position_y_m']:.1f},{metrics['target_position_z_m']:.1f})m"
        ),
        (
            f"target_vel=({metrics['target_velocity_x_mps']:.1f},"
            f"{metrics['target_velocity_y_mps']:.1f},{metrics['target_velocity_z_mps']:.1f})mps"
        ),
        (
            f"target_yaw={metrics['target_yaw_deg']:.1f} "
            f"target_fwd_src={metrics['target_forward_source']}"
        ),
        f"rel=({metrics['relative_x_m']:.1f},{metrics['relative_y_m']:.1f},{metrics['relative_z_m']:.1f})m",
        f"dist={metrics['distance_m']:.1f}m",
        f"bearing={metrics['bearing_deg']:.1f}",
        f"hdg_err={metrics['heading_error_deg']:.1f}",
        f"pitch_err={metrics['pitch_to_target_deg']:.1f}",
        f"target_course={metrics['target_course_deg']:.1f}",
    ]
    if include_anchor:
        parts.append(f"behind_err={metrics['behind_aspect_error_deg']:.1f}")
        parts.append(
            (
                f"rear_anchor=({metrics['rear_anchor_x_m']:.1f},"
                f"{metrics['rear_anchor_y_m']:.1f},{metrics['rear_anchor_z_m']:.1f})m"
            )
        )
        parts.append(f"rear_anchor_src={metrics['rear_anchor_source']}")
        parts.append(f"rear_anchor_dist={metrics['behind_anchor_distance_m']:.1f}m")
    return " ".join(parts)


def _format_guidance_debug(debug):
    if not debug:
        return ""
    return (
        f"des_hdg={debug['desired_heading_deg']:.1f} "
        f"raw_hdg_err={debug['raw_heading_error_deg']:.1f} "
        f"clamped_hdg_err={debug['heading_error_deg']:.1f} "
        f"des_pitch={debug['desired_pitch_deg']:.1f} "
        f"raw_pitch_err={debug['raw_pitch_error_deg']:.1f} "
        f"clamped_pitch_err={debug['pitch_error_deg']:.1f}"
    )


def _branch_log(node_name, branch_name, ego, command, metrics=None, guidance_debug=None, extra=None):
    parts = [
        f"{node_name} branch={branch_name}",
        f"ego_yaw={ego.yaw_deg:.1f}",
        f"ego_pitch={ego.pitch_deg:.1f}",
        f"ego_roll={ego.roll_deg:.1f}",
        f"ego_spd={ego.airspeed_mps:.1f}mps",
        f"ego_alt={altitude_m(ego):.1f}m",
    ]
    metric_text = _format_target_metrics(metrics, include_anchor=True)
    if metric_text:
        parts.append(metric_text)
    guidance_text = _format_guidance_debug(guidance_debug)
    if guidance_text:
        parts.append(guidance_text)
    if extra:
        parts.append(str(extra))
    parts.append(format_command(command))
    _debug_log(" ".join(parts))


def _log_state_quality(prefix, state):
    if not _state_is_valid(state):
        _debug_log(f"{prefix} state=invalid")
        return

    inconsistencies = []
    if (not state.on_ground) and state.airspeed_mps < 1.0 and altitude_m(state) < 25.0:
        inconsistencies.append("airborne_flag_with_zero_speed")
    if state.on_ground and altitude_m(state) > 40.0:
        inconsistencies.append("on_ground_flag_at_high_altitude")
    if abs(float(getattr(state, 'ground_speed_mps', 0.0)) - float(state.airspeed_mps)) > 120.0:
        inconsistencies.append("large_groundspeed_airspeed_gap")

    if inconsistencies:
        _debug_log(
            f"{prefix} state_quality actor_id={getattr(state, 'actor_id', 'UNBOUND')} "
            f"on_ground={state.on_ground} airspeed={state.airspeed_mps:.1f} "
            f"ground_speed={getattr(state, 'ground_speed_mps', 0.0):.1f} "
            f"alt={altitude_m(state):.1f} agl={getattr(state, 'altitude_agl_m', 0.0):.1f} "
            f"flags={','.join(inconsistencies)}"
        )


def _state_sanity_warnings(state, command=None):
    if not _state_is_valid(state):
        return ["state_invalid"]

    warnings = []
    velocity_norm = (
        (float(state.velocity_mps.x) ** 2)
        + (float(state.velocity_mps.y) ** 2)
        + (float(state.velocity_mps.z) ** 2)
    ) ** 0.5

    if (not state.on_ground) and altitude_m(state) < 25.0 and state.airspeed_mps < 1.0 and velocity_norm < 1.0:
        warnings.append("on_ground_false_but_low_alt_zero_speed")
    if state.airspeed_mps < 1.0 and velocity_norm > 15.0:
        warnings.append("airspeed_zero_but_velocity_high")
    if command is not None and float(command[0]) >= 0.85 and velocity_norm < 1.0:
        warnings.append("high_throttle_but_velocity_zero")

    return warnings


def _recovery_flags(ego, blackboard):
    previous = blackboard.get('f16_recovery_flags', {})
    stall_active_before = bool(previous.get('stalling', False))

    stall_speed_mps = float(_RECOVERY_CFG.get('stall_speed_mps', 80.0))
    stall_clear_speed_mps = float(_RECOVERY_CFG.get('stall_clear_speed_mps', 92.0))
    stall_aoa_deg = float(_RECOVERY_CFG.get('stall_aoa_deg', 14.0))
    stall_clear_aoa_deg = float(_RECOVERY_CFG.get('stall_clear_aoa_deg', 11.0))

    if stall_active_before:
        stalling = (not ego.on_ground) and (
            ego.airspeed_mps <= stall_clear_speed_mps
            or ego.angle_of_attack_deg >= stall_clear_aoa_deg
        )
    else:
        stalling = (not ego.on_ground) and (
            ego.airspeed_mps <= stall_speed_mps
            or ego.angle_of_attack_deg >= stall_aoa_deg
        )

    return {
        'stalling': stalling,
        'low_altitude': (not ego.on_ground)
        and (altitude_m(ego) <= float(_RECOVERY_CFG.get('low_altitude_m', 250.0))),
        'excessive_bank': abs(ego.bank_deg) >= float(_RECOVERY_CFG.get('max_bank_deg', 75.0)),
        'excessive_pitch': abs(ego.pitch_deg) >= float(_RECOVERY_CFG.get('max_pitch_deg', 35.0)),
    }


class F16Action(Node):
    def __init__(self, name, agent):
        super().__init__(name)
        self.type = "Action"

    def _send_command(self, agent, throttle, elevator, aileron, rudder, branch_name=None):
        if getattr(agent, 'interface', None) is None:
            self.status = Status.FAILURE
            return Status.FAILURE

        blackboard = getattr(agent, 'blackboard', {})
        command = (throttle, elevator, aileron, rudder)
        branch_name = str(branch_name or "").strip()
        _record_branch_attempt(blackboard, self.name, branch_name or "unspecified", command)
        previous_final = blackboard.get('bt_tick_final_command')
        if previous_final is not None:
            overwrite_text = (
                f"overwrite_by_other_branch prev={previous_final.get('node')}:{previous_final.get('branch')} "
                f"new={self.name}:{branch_name or 'unspecified'}"
            )
            blackboard.setdefault('bt_tick_overwrites', []).append(overwrite_text)
            _debug_log(overwrite_text)

        blackboard['bt_tick_current_branch'] = f"{self.name}:{branch_name or 'unspecified'}"
        blackboard['bt_tick_final_command'] = {
            'node': self.name,
            'branch': branch_name or 'unspecified',
            'command': {
                'throttle': round(float(throttle), 3),
                'elevator': round(float(elevator), 3),
                'aileron': round(float(aileron), 3),
                'rudder': round(float(rudder), 3),
            },
        }

        ego_state = blackboard.get('ego_state')
        sanity = _state_sanity_warnings(ego_state, command)
        if sanity:
            _debug_log(
                f"state_warning node={self.name} branch={branch_name or 'unspecified'} "
                f"warnings={sanity}"
            )

        if hasattr(agent.interface, 'set_command_context'):
            agent.interface.set_command_context(self.name, branch_name)
        if hasattr(agent.interface, 'describe_ego_binding'):
            _debug_log(
                f"CommandDispatch node={self.name} target={agent.interface.describe_ego_binding()} "
                f"branch={branch_name or 'unspecified'} {format_command(command)}"
            )

        if hasattr(agent.interface, 'send_ego_command'):
            agent.interface.send_ego_command(throttle, elevator, aileron, rudder)
        else:
            self.status = Status.FAILURE
            return Status.FAILURE

        self.status = Status.RUNNING
        return self.status


class IsAirborne(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            return Status.FAILURE

        is_airborne = _is_airborne_now(ego)
        return Status.SUCCESS if is_airborne else Status.FAILURE


class NeedsTakeoff(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            _condition_log(blackboard, self.name, "FAILURE", "ego_state_invalid")
            return Status.FAILURE
        if _force_takeoff_active(blackboard):
            reason = "force_takeoff_window_active"
            _condition_log(blackboard, self.name, "SUCCESS", reason)
            return Status.SUCCESS

        takeoff_needed = _needs_takeoff(ego, blackboard)
        reason = (
            f"on_ground={ego.on_ground} airborne_now={_is_airborne_now(ego)} "
            f"takeoff_exit_reached={_takeoff_exit_reached(ego)} "
            f"alt={altitude_m(ego):.1f} airspeed={ego.airspeed_mps:.1f} "
            f"takeoff_exit_alt={_takeoff_exit_altitude_m():.1f} "
            f"takeoff_exit_spd={_takeoff_exit_speed_mps():.1f}"
        )
        _condition_log(blackboard, self.name, "SUCCESS" if takeoff_needed else "FAILURE", reason)
        return Status.SUCCESS if takeoff_needed else Status.FAILURE


class NeedsClimbToSafeAltitude(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            _condition_log(blackboard, self.name, "FAILURE", "ego_state_invalid")
            return Status.FAILURE
        if _force_takeoff_active(blackboard):
            _condition_log(blackboard, self.name, "FAILURE", "force_takeoff_window_active")
            return Status.FAILURE
        result = _needs_safe_altitude(ego)
        reason = f"airborne_now={_is_airborne_now(ego)} alt={altitude_m(ego):.1f} safe_alt={float(_AI_CFG.get('safe_altitude_m', 1500.0)):.1f}"
        _condition_log(blackboard, self.name, "SUCCESS" if result else "FAILURE", reason)
        return Status.SUCCESS if result else Status.FAILURE


class IsStalling(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            _condition_log(blackboard, self.name, "FAILURE", "ego_state_invalid")
            return Status.FAILURE
        if _force_takeoff_active(blackboard):
            _condition_log(blackboard, self.name, "FAILURE", "force_takeoff_window_active")
            return Status.FAILURE

        flags = _recovery_flags(ego, blackboard)
        blackboard['f16_recovery_flags'] = flags
        _condition_log(blackboard, self.name, "SUCCESS" if any(flags.values()) else "FAILURE", str(flags))
        return Status.SUCCESS if any(flags.values()) else Status.FAILURE


class TargetLost(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        _, target = _get_states(agent, blackboard)
        if _state_is_valid(target):
            _snapshot_target_track(blackboard, target)
            return Status.FAILURE
        return Status.SUCCESS if _track_is_recent(blackboard) else Status.FAILURE


class TargetFar(SyncCondition):
    def __init__(self, name, agent, range_m=None):
        super().__init__(name, self._check)
        self.range_m = float(range_m) if range_m is not None else None

    def _check(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            return Status.FAILURE

        metrics = _update_target_metrics(blackboard, ego, target)
        threshold = self.range_m if self.range_m is not None else _far_range_m()
        return Status.SUCCESS if metrics['distance_m'] >= threshold else Status.FAILURE


class TargetInRange(SyncCondition):
    def __init__(self, name, agent, range_m=None):
        super().__init__(name, self._check)
        self.range_m = float(range_m) if range_m is not None else None

    def _check(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            return Status.FAILURE

        metrics = _update_target_metrics(blackboard, ego, target)
        threshold = self.range_m if self.range_m is not None else _in_range_m()
        return Status.SUCCESS if metrics['distance_m'] <= threshold else Status.FAILURE


class IsBehindTarget(SyncCondition):
    def __init__(self, name, agent):
        super().__init__(name, self._check)

    def _check(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            return Status.FAILURE

        metrics = _update_target_metrics(blackboard, ego, target)
        aspect_threshold = _rear_aspect_angle_deg()
        anchor_tolerance_m = float(_FOLLOW_CFG.get('anchor_tolerance_m', 250.0))
        is_behind = (
            metrics['behind_aspect_error_deg'] <= aspect_threshold
            and metrics['behind_anchor_distance_m'] <= anchor_tolerance_m
        )
        return Status.SUCCESS if is_behind else Status.FAILURE


class Takeoff(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            self.status = Status.FAILURE
            return self.status

        if _takeoff_exit_reached(ego):
            blackboard['f16_takeoff_committed'] = False
            blackboard['f16_takeoff_active_ticks'] = 0
            self.status = Status.SUCCESS
            return self.status

        blackboard['f16_takeoff_committed'] = True
        blackboard['f16_takeoff_active_ticks'] = int(blackboard.get('f16_takeoff_active_ticks', 0)) + 1
        _log_state_quality("Takeoff", ego)
        if _force_takeoff_active(blackboard):
            branch, command = _forced_takeoff_phase_command(ego)
        else:
            branch, command = _takeoff_phase_command(ego)
        _branch_log(
            self.name,
            branch,
            ego,
            command,
            extra=(
                f"takeoff_exit_alt={_takeoff_exit_altitude_m():.1f}m "
                f"takeoff_exit_spd={_takeoff_exit_speed_mps():.1f}mps on_ground={ego.on_ground} "
                f"ground_speed={getattr(ego, 'ground_speed_mps', 0.0):.1f}mps "
                f"takeoff_ticks={blackboard.get('f16_takeoff_active_ticks', 0)} "
                f"force_takeoff={blackboard.get('f16_force_takeoff_active', False)}"
            ),
        )
        return self._send_command(agent, *command, branch_name=branch)


class ClimbToSafeAltitude(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            self.status = Status.FAILURE
            return self.status

        if _state_is_valid(target):
            _snapshot_target_track(blackboard, target)

        current_altitude = altitude_m(ego)
        safe_altitude = float(_AI_CFG.get('safe_altitude_m', 1500.0))
        if current_altitude >= safe_altitude:
            self.status = Status.SUCCESS
            return self.status

        if not _takeoff_exit_reached(ego):
            blackboard['f16_takeoff_committed'] = True
            branch, command = _takeoff_phase_command(ego)
            _branch_log(
                self.name,
                f"protected_{branch}",
                ego,
                command,
                extra=(
                    f"safe_alt={safe_altitude:.1f}m "
                    f"takeoff_exit_spd={_takeoff_exit_speed_mps():.1f}mps"
                ),
            )
            return self._send_command(agent, *command, branch_name=f"protected_{branch}")

        runway_heading = float(_TAKEOFF_CFG.get('runway_heading_deg', ego.yaw_deg))
        turn_initiation_altitude_m = float(_CLIMB_CFG.get('turn_initiation_altitude_m', 400.0))
        low_alt_turn_limit_deg = float(_CLIMB_CFG.get('max_heading_change_deg_below_turn_altitude', 10.0))
        climb_turn_limit_deg = float(_CLIMB_CFG.get('max_heading_change_deg_during_climb', 20.0))

        desired_heading = runway_heading
        heading_mode = "runway_hold"
        if _state_is_valid(target):
            target_heading = bearing_deg(point_from_state(ego), point_from_state(target))
            if current_altitude < turn_initiation_altitude_m:
                desired_heading = runway_heading
                heading_mode = "safe_climb_hold"
            else:
                allowed_delta = (
                    low_alt_turn_limit_deg
                    if current_altitude < (0.5 * safe_altitude)
                    else climb_turn_limit_deg
                )
                desired_heading = limited_heading_toward(ego.yaw_deg, target_heading, allowed_delta)
                heading_mode = "limited_turn_to_target"

        throttle = float(_CLIMB_CFG.get('throttle', 1.0))
        desired_pitch = float(_CLIMB_CFG.get('climb_pitch_deg', 16.0))
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=throttle,
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        metrics = _update_target_metrics(blackboard, ego, target) if _state_is_valid(target) else None
        _branch_log(
            self.name,
            heading_mode,
            ego,
            command,
            metrics=metrics,
            guidance_debug=guidance_debug,
            extra=f"safe_alt={safe_altitude:.1f}m",
        )
        return self._send_command(agent, *command, branch_name=heading_mode)


class InterceptTarget(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            self.status = Status.FAILURE
            return self.status

        _log_state_quality("InterceptTarget.ego", ego)
        _log_state_quality("InterceptTarget.enemy", target)
        metrics = _update_target_metrics(blackboard, ego, target)
        aim_point = lead_intercept_point(
            ego,
            target,
            min_lookahead_s=float(_INTERCEPT_CFG.get('min_lookahead_s', 0.5)),
            max_lookahead_s=float(_INTERCEPT_CFG.get('max_lookahead_s', 6.0)),
            min_closure_speed_mps=float(_INTERCEPT_CFG.get('min_closure_speed_mps', 120.0)),
        )
        desired_heading = bearing_deg(point_from_state(ego), aim_point)
        desired_pitch = clamp(
            pitch_deg(point_from_state(ego), aim_point),
            -10.0,
            float(_INTERCEPT_CFG.get('max_pitch_deg', 18.0)),
        )
        throttle = float(_INTERCEPT_CFG.get('throttle', 1.0))
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=throttle,
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        _branch_log(
            self.name,
            "lead_intercept",
            ego,
            command,
            metrics=metrics,
            guidance_debug=guidance_debug,
            extra=f"aim_point=({aim_point['x']:.1f},{aim_point['y']:.1f},{aim_point['z']:.1f})",
        )
        return self._send_command(agent, *command, branch_name="lead_intercept")


class GetBehindTarget(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            self.status = Status.FAILURE
            return self.status

        _log_state_quality("GetBehindTarget.ego", ego)
        _log_state_quality("GetBehindTarget.enemy", target)
        metrics = _update_target_metrics(blackboard, ego, target)
        anchor_solution = rear_anchor_solution(
            target,
            distance_back_m=float(_GET_BEHIND_CFG.get('distance_m', 900.0)),
            lookahead_s=float(_GET_BEHIND_CFG.get('lookahead_s', 1.4)),
            altitude_offset_m=float(_GET_BEHIND_CFG.get('altitude_offset_m', 0.0)),
            min_velocity_mps=float(_GET_BEHIND_CFG.get('min_target_forward_speed_mps', 25.0)),
        )
        anchor = anchor_solution['anchor']
        desired_heading = bearing_deg(point_from_state(ego), anchor)
        desired_pitch = clamp(
            pitch_deg(point_from_state(ego), anchor),
            -12.0,
            float(_GET_BEHIND_CFG.get('max_pitch_deg', 20.0)),
        )

        throttle = throttle_for_spacing(
            distance_error_m=metrics['behind_anchor_distance_m'],
            speed_error_mps=target.airspeed_mps - ego.airspeed_mps,
            base_throttle=float(_GET_BEHIND_CFG.get('base_throttle', 0.95)),
            distance_gain=float(_GET_BEHIND_CFG.get('distance_gain', 0.00012)),
            speed_gain=float(_GET_BEHIND_CFG.get('speed_gain', 0.003)),
            min_throttle=float(_GET_BEHIND_CFG.get('min_throttle', 0.70)),
            max_throttle=float(_GET_BEHIND_CFG.get('max_throttle', 1.0)),
        )
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=throttle,
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        _branch_log(
            self.name,
            "rear_positioning",
            ego,
            command,
            metrics=metrics,
            guidance_debug=guidance_debug,
            extra=(
                f"anchor_point=({anchor['x']:.1f},{anchor['y']:.1f},{anchor['z']:.1f}) "
                f"anchor_src={anchor_solution['source']} "
                f"spacing_thr={throttle:.2f}"
            ),
        )
        return self._send_command(agent, *command, branch_name="rear_positioning")


class FollowTarget(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not (_state_is_valid(ego) and _state_is_valid(target)):
            self.status = Status.FAILURE
            return self.status

        _log_state_quality("FollowTarget.ego", ego)
        _log_state_quality("FollowTarget.enemy", target)
        metrics = _update_target_metrics(blackboard, ego, target)
        anchor_solution = rear_anchor_solution(
            target,
            distance_back_m=_follow_distance_m(),
            lookahead_s=float(_FOLLOW_CFG.get('lookahead_s', 0.8)),
            altitude_offset_m=float(_FOLLOW_CFG.get('altitude_offset_m', 0.0)),
            min_velocity_mps=float(_FOLLOW_CFG.get('min_target_forward_speed_mps', 25.0)),
        )
        anchor = anchor_solution['anchor']
        desired_heading = bearing_deg(point_from_state(ego), anchor)
        desired_pitch = clamp(
            pitch_deg(point_from_state(ego), anchor),
            -10.0,
            float(_FOLLOW_CFG.get('max_pitch_deg', 16.0)),
        )

        distance_error_m = metrics['behind_anchor_distance_m'] - float(_FOLLOW_CFG.get('anchor_tolerance_m', 250.0))
        throttle = throttle_for_spacing(
            distance_error_m=distance_error_m,
            speed_error_mps=target.airspeed_mps - ego.airspeed_mps,
            base_throttle=float(_FOLLOW_CFG.get('base_throttle', 0.82)),
            distance_gain=float(_FOLLOW_CFG.get('distance_gain', 0.00010)),
            speed_gain=float(_FOLLOW_CFG.get('speed_gain', 0.0035)),
            min_throttle=float(_FOLLOW_CFG.get('min_throttle', 0.55)),
            max_throttle=float(_FOLLOW_CFG.get('max_throttle', 1.0)),
        )
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=throttle,
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        _branch_log(
            self.name,
            "rear_follow",
            ego,
            command,
            metrics=metrics,
            guidance_debug=guidance_debug,
            extra=(
                f"anchor_point=({anchor['x']:.1f},{anchor['y']:.1f},{anchor['z']:.1f}) "
                f"anchor_src={anchor_solution['source']} "
                f"spacing_error={distance_error_m:.1f}m spacing_thr={throttle:.2f}"
            ),
        )
        return self._send_command(agent, *command, branch_name="rear_follow")


class ReacquireTarget(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            self.status = Status.FAILURE
            return self.status

        if _state_is_valid(target):
            self.status = Status.SUCCESS
            return self.status

        track = _get_last_target_track(blackboard)
        if not track or not _track_is_recent(blackboard):
            self.status = Status.FAILURE
            return self.status

        track_age_s = _track_age_s(track)
        prediction_s = min(
            track_age_s + float(_REACQUIRE_CFG.get('lookahead_s', 2.0)),
            float(_REACQUIRE_CFG.get('max_prediction_s', 8.0)),
        )
        reacquire_point = {
            'x': float(track['position']['x'] + (track['velocity']['x'] * prediction_s)),
            'y': float(track['position']['y'] + (track['velocity']['y'] * prediction_s)),
            'z': float(track['position']['z'] + (track['velocity']['z'] * prediction_s)),
        }
        desired_heading = bearing_deg(point_from_state(ego), reacquire_point)
        desired_pitch = clamp(
            pitch_deg(point_from_state(ego), reacquire_point),
            -8.0,
            float(_REACQUIRE_CFG.get('max_pitch_deg', 15.0)),
        )
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=float(_REACQUIRE_CFG.get('throttle', 0.90)),
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        predicted_metrics = {
            'relative_x_m': reacquire_point['x'] - ego.position_m.x,
            'relative_y_m': reacquire_point['y'] - ego.position_m.y,
            'relative_z_m': reacquire_point['z'] - ego.position_m.z,
            'distance_m': distance_m(point_from_state(ego), reacquire_point),
            'bearing_deg': desired_heading,
            'heading_error_deg': wrap_angle_deg(desired_heading - ego.yaw_deg),
            'pitch_to_target_deg': pitch_deg(point_from_state(ego), reacquire_point),
            'target_position_x_m': float(track['position']['x']),
            'target_position_y_m': float(track['position']['y']),
            'target_position_z_m': float(track['position']['z']),
            'target_velocity_x_mps': float(track['velocity']['x']),
            'target_velocity_y_mps': float(track['velocity']['y']),
            'target_velocity_z_mps': float(track['velocity']['z']),
            'target_yaw_deg': float(track.get('yaw_deg', track.get('course_deg', 0.0))),
            'target_pitch_deg': float(track.get('pitch_deg', 0.0)),
            'target_roll_deg': float(track.get('roll_deg', 0.0)),
            'target_course_deg': float(track.get('course_deg', 0.0)),
            'target_forward_source': 'last_track',
            'target_forward_x': 0.0,
            'target_forward_y': 0.0,
            'behind_aspect_error_deg': 0.0,
            'behind_anchor_distance_m': 0.0,
            'rear_anchor_x_m': float(reacquire_point['x']),
            'rear_anchor_y_m': float(reacquire_point['y']),
            'rear_anchor_z_m': float(reacquire_point['z']),
            'rear_anchor_source': 'last_track_prediction',
            'rear_anchor_course_deg': float(track.get('course_deg', 0.0)),
        }
        _branch_log(
            self.name,
            "last_known_track",
            ego,
            command,
            metrics=predicted_metrics,
            guidance_debug=guidance_debug,
            extra=(
                f"track_age={track_age_s:.2f}s prediction_s={prediction_s:.2f} "
                f"track_pos=({track['position']['x']:.1f},{track['position']['y']:.1f},{track['position']['z']:.1f}) "
                f"track_vel=({track['velocity']['x']:.1f},{track['velocity']['y']:.1f},{track['velocity']['z']:.1f})"
            ),
        )
        return self._send_command(agent, *command, branch_name="last_known_track")


class Recover(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, _ = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            self.status = Status.FAILURE
            return self.status

        flags = _recovery_flags(ego, blackboard)
        blackboard['f16_recovery_flags'] = flags
        if not any(flags.values()):
            _debug_log(
                f"Recover branch=clear ego_yaw={ego.yaw_deg:.1f} ego_pitch={ego.pitch_deg:.1f} "
                f"ego_roll={ego.roll_deg:.1f} ego_spd={ego.airspeed_mps:.1f}mps "
                f"ego_alt={altitude_m(ego):.1f}m aoa={ego.angle_of_attack_deg:.1f}deg bank={ego.bank_deg:.1f}"
            )
            self.status = Status.SUCCESS
            return self.status

        throttle = 1.0
        desired_pitch = 0.0
        branch = "attitude_recovery"

        if flags['stalling']:
            branch = "stall_unload"
            desired_pitch = float(_RECOVERY_CFG.get('stall_nose_down_pitch_deg', -8.0))
            if flags['low_altitude']:
                desired_pitch = float(_RECOVERY_CFG.get('stall_low_altitude_pitch_deg', -2.0))
            throttle = float(_RECOVERY_CFG.get('stall_throttle', 1.0))
        elif flags['low_altitude']:
            branch = "low_altitude_escape"
            desired_pitch = float(_RECOVERY_CFG.get('low_altitude_pitch_deg', 10.0))
        elif flags['excessive_pitch'] and ego.pitch_deg > 0.0:
            branch = "pitch_unload"
            desired_pitch = float(_RECOVERY_CFG.get('pitch_unload_deg', -3.0))

        if flags['excessive_pitch'] and ego.pitch_deg > 0.0:
            desired_pitch = min(desired_pitch, float(_RECOVERY_CFG.get('pitch_unload_deg', -3.0)))

        elevator = clamp(
            ((desired_pitch - ego.pitch_deg) * 0.10)
            - (ego.angle_of_attack_deg * float(_RECOVERY_CFG.get('aoa_to_elevator_gain', 0.03))),
            -float(_RECOVERY_CFG.get('max_elevator_cmd', 0.45)),
            float(_RECOVERY_CFG.get('max_elevator_cmd', 0.45)),
        )
        aileron = clamp(
            (-ego.roll_deg) * float(_RECOVERY_CFG.get('bank_level_gain', 0.05)),
            -float(_RECOVERY_CFG.get('max_aileron_cmd', 0.45)),
            float(_RECOVERY_CFG.get('max_aileron_cmd', 0.45)),
        )
        rudder = clamp(
            (-ego.roll_deg) * float(_RECOVERY_CFG.get('rudder_level_gain', 0.012)),
            -float(_RECOVERY_CFG.get('max_rudder_cmd', 0.25)),
            float(_RECOVERY_CFG.get('max_rudder_cmd', 0.25)),
        )
        command = (throttle, elevator, aileron, rudder)
        _branch_log(
            self.name,
            branch,
            ego,
            command,
            extra=(
                f"aoa={ego.angle_of_attack_deg:.1f} desired_pitch={desired_pitch:.1f} "
                f"flags={flags}"
            ),
        )
        return self._send_command(agent, *command, branch_name=branch)


class Reposition(F16Action):
    def __init__(self, name, agent):
        super().__init__(name, agent)

    async def run(self, agent, blackboard):
        ego, target = _get_states(agent, blackboard)
        if not _state_is_valid(ego):
            self.status = Status.FAILURE
            return self.status

        desired_heading = ego.yaw_deg
        if _state_is_valid(target):
            _snapshot_target_track(blackboard, target)
            desired_heading = bearing_deg(point_from_state(ego), point_from_state(target))

        safe_altitude = float(_AI_CFG.get('safe_altitude_m', 1500.0))
        altitude_error = safe_altitude - altitude_m(ego)
        desired_pitch = clamp(altitude_error / 120.0, -6.0, 12.0)
        command, guidance_debug = build_guidance_command(
            ego,
            desired_heading_deg=desired_heading,
            desired_pitch_deg=desired_pitch,
            throttle=0.78,
            controller_cfg=_CONTROLLER_CFG,
            return_debug=True,
        )
        metrics = _update_target_metrics(blackboard, ego, target) if _state_is_valid(target) else None
        _branch_log(
            self.name,
            "safe_reposition",
            ego,
            command,
            metrics=metrics,
            guidance_debug=guidance_debug,
            extra=f"safe_alt={safe_altitude:.1f}m",
        )
        return self._send_command(agent, *command, branch_name="safe_reposition")
