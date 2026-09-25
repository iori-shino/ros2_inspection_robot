"""Small geometry helpers independent of ROS 2."""

import math


def compute_target_error(x, y, theta, target_x, target_y):
    """Return distance and signed heading error to a target.

    Angles are in radians. Positive heading error means turn left;
    negative means turn right. At the target, both errors are zero.
    """
    dx = target_x - x
    dy = target_y - y
    distance = math.hypot(dx, dy)

    if distance == 0.0:
        return 0.0, 0.0

    target_heading = math.atan2(dy, dx)
    raw_error = target_heading - theta
    heading_error = math.atan2(math.sin(raw_error), math.cos(raw_error))

    return distance, heading_error
