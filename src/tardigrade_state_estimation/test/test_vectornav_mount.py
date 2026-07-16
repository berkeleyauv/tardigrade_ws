# Copyright 2026 Berkeley AUV

import math
import unittest

from geometry_msgs.msg import Quaternion, Vector3

from tardigrade_state_estimation.vectornav_odometry import (
    quaternion_ned_sensor_to_enu_robot,
    vector_sensor_frd_to_robot_flu,
)


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class VectorNavMountTest(unittest.TestCase):
    def test_sensor_axes_map_to_robot_flu(self):
        sensor_vector = Vector3(x=2.0, y=-3.0, z=4.0)

        robot_vector = vector_sensor_frd_to_robot_flu(sensor_vector)

        self.assertEqual(robot_vector.x, 3.0)
        self.assertEqual(robot_vector.y, -2.0)
        self.assertEqual(robot_vector.z, -4.0)

    def test_level_robot_pointing_north_has_enu_yaw_ninety(self):
        # With the robot pointing north, the mounted sensor +X points east.
        sensor_orientation = Quaternion(
            z=math.sqrt(0.5),
            w=math.sqrt(0.5),
        )

        robot_orientation = quaternion_ned_sensor_to_enu_robot(
            sensor_orientation
        )

        self.assertTrue(
            math.isclose(
                yaw_from_quaternion(robot_orientation),
                math.pi / 2.0,
                abs_tol=1e-7,
            )
        )

    def test_level_robot_pointing_east_has_zero_enu_yaw(self):
        sensor_orientation = Quaternion(z=1.0, w=0.0)

        robot_orientation = quaternion_ned_sensor_to_enu_robot(
            sensor_orientation
        )

        self.assertTrue(
            math.isclose(
                yaw_from_quaternion(robot_orientation),
                0.0,
                abs_tol=1e-7,
            )
        )


if __name__ == '__main__':
    unittest.main()
