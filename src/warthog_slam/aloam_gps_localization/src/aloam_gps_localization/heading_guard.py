#!/usr/bin/env python3

import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def path_heading_error(path_points, robot_x, robot_y, robot_yaw, lookahead_distance):
    """Return the heading error to a look-ahead point on an XY path."""
    if not path_points:
        return None

    closest_index = min(
        range(len(path_points)),
        key=lambda index: (
            (path_points[index][0] - robot_x) ** 2
            + (path_points[index][1] - robot_y) ** 2
        ),
    )

    target_index = closest_index
    travelled = 0.0
    while target_index + 1 < len(path_points) and travelled < lookahead_distance:
        current = path_points[target_index]
        following = path_points[target_index + 1]
        travelled += math.hypot(following[0] - current[0], following[1] - current[1])
        target_index += 1

    target_x, target_y = path_points[target_index]
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    if math.hypot(delta_x, delta_y) < 1e-6:
        return None

    desired_yaw = math.atan2(delta_y, delta_x)
    return normalize_angle(desired_yaw - robot_yaw)


def turn_speed(heading_error, minimum_speed, maximum_speed, gain):
    """Return a signed proportional in-place turn speed with a deadband floor."""
    magnitude = min(maximum_speed, max(minimum_speed, gain * abs(heading_error)))
    return math.copysign(magnitude, heading_error)


def rate_limit(current, target, maximum_delta):
    """Move current toward target by at most maximum_delta."""
    if target > current:
        return min(target, current + maximum_delta)
    return max(target, current - maximum_delta)
