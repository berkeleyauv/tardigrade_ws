import math
import os
import time

os.environ.setdefault('MAVLINK20', '1')

from pymavlink import mavutil

mavutil.set_dialect('common')


def connect_mavlink(device, baudrate, source_system=42, source_component=191):
    return mavutil.mavlink_connection(
        device,
        baud=baudrate,
        source_system=source_system,
        source_component=source_component,
        autoreconnect=True,
    )


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b

    return [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]


def enu_to_ned_point(p):
    return [p.y, p.x, -p.z]


def flu_to_frd_vector(v):
    return [v.x, -v.y, -v.z]


def quaternion_enu_flu_to_ned_frd(q):
    ros_q = [q.w, q.x, q.y, q.z]

    q_ned_from_enu = [
        0.0,
        math.sqrt(0.5),
        math.sqrt(0.5),
        0.0,
    ]

    q_flu_from_frd = [
        0.0,
        1.0,
        0.0,
        0.0,
    ]

    return quat_multiply(
        quat_multiply(q_ned_from_enu, ros_q),
        q_flu_from_frd,
    )


def now_us():
    return int(time.time() * 1_000_000)
