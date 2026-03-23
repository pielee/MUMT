import math


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def wrap_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def horizontal_speed_mps(state):
    return math.hypot(state.velocity_mps.x, state.velocity_mps.y)


def distance_m(a, b):
    dx = b['x'] - a['x']
    dy = b['y'] - a['y']
    dz = b['z'] - a['z']
    return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))


def point_from_state(state):
    return {
        'x': float(state.position_m.x),
        'y': float(state.position_m.y),
        'z': float(state.position_m.z),
    }


def bearing_deg(a, b):
    return math.degrees(math.atan2(b['y'] - a['y'], b['x'] - a['x']))


def pitch_deg(a, b):
    dx = b['x'] - a['x']
    dy = b['y'] - a['y']
    horizontal_distance = max(1.0, math.hypot(dx, dy))
    altitude_delta = b['z'] - a['z']
    return math.degrees(math.atan2(altitude_delta, horizontal_distance))


def forward_vector_xy_from_state(state, min_velocity_mps=25.0, return_meta=False):
    speed = horizontal_speed_mps(state)
    if speed >= min_velocity_mps:
        forward_x = float(state.velocity_mps.x) / max(speed, 1e-6)
        forward_y = float(state.velocity_mps.y) / max(speed, 1e-6)
        source = "velocity"
        course = math.degrees(math.atan2(forward_y, forward_x))
    else:
        course = float(getattr(state, 'yaw_deg', 0.0))
        course_rad = math.radians(course)
        forward_x = math.cos(course_rad)
        forward_y = math.sin(course_rad)
        source = "yaw"

    if not return_meta:
        return forward_x, forward_y

    return {
        'forward_x': float(forward_x),
        'forward_y': float(forward_y),
        'source': source,
        'course_deg': float(course),
        'horizontal_speed_mps': float(speed),
    }


def course_deg_from_state(state, min_velocity_mps=25.0):
    forward = forward_vector_xy_from_state(
        state,
        min_velocity_mps=min_velocity_mps,
        return_meta=True,
    )
    return float(forward['course_deg'])


def rear_anchor_solution(
    target,
    distance_back_m,
    lookahead_s=0.0,
    altitude_offset_m=0.0,
    min_velocity_mps=25.0,
):
    forward = forward_vector_xy_from_state(
        target,
        min_velocity_mps=min_velocity_mps,
        return_meta=True,
    )
    lookahead_point = {
        'x': float(target.position_m.x + (target.velocity_mps.x * lookahead_s)),
        'y': float(target.position_m.y + (target.velocity_mps.y * lookahead_s)),
        'z': float(target.position_m.z + (target.velocity_mps.z * lookahead_s) + altitude_offset_m),
    }
    anchor = {
        'x': lookahead_point['x'] - (forward['forward_x'] * distance_back_m),
        'y': lookahead_point['y'] - (forward['forward_y'] * distance_back_m),
        'z': lookahead_point['z'],
    }
    return {
        'anchor': anchor,
        'lookahead_point': lookahead_point,
        'forward_x': float(forward['forward_x']),
        'forward_y': float(forward['forward_y']),
        'source': str(forward['source']),
        'course_deg': float(forward['course_deg']),
        'horizontal_speed_mps': float(forward['horizontal_speed_mps']),
    }


def behind_target_point(
    target,
    distance_back_m,
    lookahead_s=0.0,
    altitude_offset_m=0.0,
    min_velocity_mps=25.0,
):
    return rear_anchor_solution(
        target,
        distance_back_m=distance_back_m,
        lookahead_s=lookahead_s,
        altitude_offset_m=altitude_offset_m,
        min_velocity_mps=min_velocity_mps,
    )['anchor']


def lead_intercept_point(
    ego,
    target,
    min_lookahead_s=0.5,
    max_lookahead_s=6.0,
    min_closure_speed_mps=120.0,
):
    ego_point = point_from_state(ego)
    target_point = point_from_state(target)
    range_m = distance_m(ego_point, target_point)
    closure_speed = max(min_closure_speed_mps, float(ego.airspeed_mps))
    lookahead_s = clamp(range_m / closure_speed, min_lookahead_s, max_lookahead_s)

    return {
        'x': float(target.position_m.x + (target.velocity_mps.x * lookahead_s)),
        'y': float(target.position_m.y + (target.velocity_mps.y * lookahead_s)),
        'z': float(target.position_m.z + (target.velocity_mps.z * lookahead_s)),
    }


def behind_aspect_error_deg(ego, target, min_velocity_mps=25.0):
    target_course = course_deg_from_state(target, min_velocity_mps=min_velocity_mps)
    target_point = point_from_state(target)
    ego_point = point_from_state(ego)
    target_to_ego_bearing = bearing_deg(target_point, ego_point)
    return abs(wrap_angle_deg(target_to_ego_bearing - (target_course + 180.0)))
