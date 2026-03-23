from .geometry import clamp, wrap_angle_deg


def altitude_m(state):
    if hasattr(state, 'altitude_world_m') and state.altitude_world_m is not None:
        return float(state.altitude_world_m)
    if hasattr(state, 'altitude_m'):
        return float(state.altitude_m)
    return float(state.position_m.z)


def build_guidance_command(
    ego,
    desired_heading_deg,
    desired_pitch_deg,
    throttle,
    controller_cfg,
    return_debug=False,
):
    raw_heading_error_deg = wrap_angle_deg(desired_heading_deg - ego.yaw_deg)
    raw_pitch_error_deg = desired_pitch_deg - ego.pitch_deg

    heading_error_deg = raw_heading_error_deg
    pitch_error_deg = raw_pitch_error_deg

    heading_error_deg = clamp(
        heading_error_deg,
        -float(controller_cfg.get('max_heading_error_deg', 35.0)),
        float(controller_cfg.get('max_heading_error_deg', 35.0)),
    )
    pitch_error_deg = clamp(
        pitch_error_deg,
        -float(controller_cfg.get('max_pitch_error_deg', 12.0)),
        float(controller_cfg.get('max_pitch_error_deg', 12.0)),
    )

    raw_aileron = (
        heading_error_deg * float(controller_cfg.get('heading_to_aileron_gain', 0.03))
        - ego.roll_deg * float(controller_cfg.get('roll_damping_gain', 0.02))
    )
    raw_elevator = pitch_error_deg * float(controller_cfg.get('pitch_to_elevator_gain', 0.06))
    raw_rudder = heading_error_deg * float(controller_cfg.get('rudder_gain', 0.012))

    max_aileron_cmd = float(controller_cfg.get('max_aileron_cmd', 0.45))
    max_elevator_cmd = float(controller_cfg.get('max_elevator_cmd', 0.40))
    max_rudder_cmd = float(controller_cfg.get('max_rudder_cmd', 0.25))

    command = (
        clamp(float(throttle), 0.0, 1.0),
        clamp(raw_elevator, -max_elevator_cmd, max_elevator_cmd),
        clamp(raw_aileron, -max_aileron_cmd, max_aileron_cmd),
        clamp(raw_rudder, -max_rudder_cmd, max_rudder_cmd),
    )
    if not return_debug:
        return command

    return command, {
        'desired_heading_deg': float(desired_heading_deg),
        'desired_pitch_deg': float(desired_pitch_deg),
        'raw_heading_error_deg': float(raw_heading_error_deg),
        'heading_error_deg': float(heading_error_deg),
        'raw_pitch_error_deg': float(raw_pitch_error_deg),
        'pitch_error_deg': float(pitch_error_deg),
        'raw_aileron': float(raw_aileron),
        'raw_elevator': float(raw_elevator),
        'raw_rudder': float(raw_rudder),
        'max_aileron_cmd': max_aileron_cmd,
        'max_elevator_cmd': max_elevator_cmd,
        'max_rudder_cmd': max_rudder_cmd,
    }


def limited_heading_toward(current_heading_deg, desired_heading_deg, max_delta_deg):
    return current_heading_deg + clamp(
        wrap_angle_deg(desired_heading_deg - current_heading_deg),
        -abs(max_delta_deg),
        abs(max_delta_deg),
    )


def throttle_for_spacing(
    distance_error_m,
    speed_error_mps,
    base_throttle,
    distance_gain,
    speed_gain,
    min_throttle,
    max_throttle,
):
    throttle = (
        float(base_throttle)
        + (float(distance_error_m) * float(distance_gain))
        + (float(speed_error_mps) * float(speed_gain))
    )
    return clamp(throttle, float(min_throttle), float(max_throttle))


def format_command(command):
    return (
        f"thr={command[0]:.2f} ele={command[1]:.2f} "
        f"ail={command[2]:.2f} rud={command[3]:.2f}"
    )
