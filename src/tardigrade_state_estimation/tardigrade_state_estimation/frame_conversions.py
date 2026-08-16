"""Coordinate conversions shared by the Tardigrade state-estimation nodes."""

import math


Q_ENU_FROM_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)


def normalize_quaternion(q):
    """Return a normalized ``(w, x, y, z)`` quaternion."""
    norm = math.sqrt(sum(component * component for component in q))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError('quaternion must be finite and nonzero')
    return tuple(component / norm for component in q)


def quaternion_multiply(a, b):
    """Multiply two ``(w, x, y, z)`` quaternions."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quaternion_inverse(q):
    """Invert a quaternion after normalizing it."""
    w, x, y, z = normalize_quaternion(q)
    return (w, -x, -y, -z)


def rotate_vector(q, vector):
    """Rotate a three-vector by quaternion ``q``."""
    q = normalize_quaternion(q)
    rotated = quaternion_multiply(
        quaternion_multiply(q, (0.0, *vector)),
        quaternion_inverse(q),
    )
    return rotated[1:]


def native_vector_to_base(vector, q_base_from_sensor):
    """Rotate a VectorNav FRD sensor vector into robot ``base_link`` FLU."""
    return rotate_vector(q_base_from_sensor, vector)


def native_orientation_to_enu_base(q_ned_from_sensor, q_base_from_sensor):
    """Convert VectorNav NED/FRD attitude into ENU/``base_link`` attitude."""
    return normalize_quaternion(quaternion_multiply(
        quaternion_multiply(Q_ENU_FROM_NED, q_ned_from_sensor),
        quaternion_inverse(q_base_from_sensor),
    ))
