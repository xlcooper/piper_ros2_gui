#!/usr/bin/env python3
"""Small non-hardware tests for the GUI's safety helpers."""

import math
import unittest

import numpy as np
from sensor_msgs.msg import JointState

from arm_simulation import gripper_transforms, link_transforms
from piper_gui import (
    bounded_increment_target,
    ordered_joint_values,
    quaternion_to_euler_degrees,
    validate_gripper_target,
    validate_joint_target,
)


class JointHelpersTest(unittest.TestCase):
    def test_orders_named_feedback(self):
        msg = JointState()
        msg.name = ["joint3", "joint1", "joint6", "joint5", "joint2", "joint4"]
        msg.position = [3.0, 1.0, 6.0, 5.0, 2.0, 4.0]
        self.assertEqual(ordered_joint_values(msg), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_rejects_incomplete_feedback(self):
        msg = JointState()
        msg.name = ["joint1"]
        msg.position = [0.0]
        self.assertIsNone(ordered_joint_values(msg))

    def test_accepts_small_target(self):
        current = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        target = list(current)
        target[0] += math.radians(2.0)
        valid, reason, _ = validate_joint_target(current, target)
        self.assertTrue(valid, reason)

    def test_accepts_large_target_change_within_joint_range(self):
        current = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        target = list(current)
        target[0] += math.radians(21.0)
        valid, reason, max_delta = validate_joint_target(current, target)
        self.assertTrue(valid, reason)
        self.assertAlmostEqual(math.degrees(max_delta), 21.0)

    def test_accepts_twenty_degree_step(self):
        current = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        target = list(current)
        target[0] += math.radians(20.0)
        valid, reason, _ = validate_joint_target(current, target)
        self.assertTrue(valid, reason)

    def test_rejects_joint_limit(self):
        current = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        target = list(current)
        target[1] = -0.01
        valid, _, _ = validate_joint_target(current, target)
        self.assertFalse(valid)

    def test_outside_feedback_does_not_block_another_joint(self):
        current = [
            0.0,
            math.radians(-1.89),
            math.radians(2.85),
            0.0,
            0.0,
            0.0,
        ]
        target = list(current)
        target[3] = math.radians(20.0)
        valid, reason, _ = validate_joint_target(current, target)
        self.assertTrue(valid, reason)

    def test_outside_feedback_cannot_move_farther_from_range(self):
        current = [0.0, math.radians(-1.89), -0.5, 0.0, 0.0, 0.0]
        target = list(current)
        target[1] = math.radians(-2.0)
        valid, _, _ = validate_joint_target(current, target)
        self.assertFalse(valid)

    def test_increment_uses_requested_delta_inside_range(self):
        current = [0.0, 0.5, -0.5, 0.0, 0.0, 0.0]
        target, applied = bounded_increment_target(current, 0, 20.0)
        self.assertAlmostEqual(applied, 20.0)
        self.assertAlmostEqual(target[0], math.radians(20.0))
        self.assertEqual(target[1:], current[1:])

    def test_increment_is_clipped_to_remaining_range(self):
        current = [math.radians(145.0), 0.5, -0.5, 0.0, 0.0, 0.0]
        target, applied = bounded_increment_target(current, 0, 20.0)
        self.assertAlmostEqual(applied, 5.0, places=5)
        self.assertAlmostEqual(target[0], math.radians(150.0), places=6)

    def test_negative_increment_is_clipped_to_lower_range(self):
        current = [0.0, math.radians(6.0), -0.5, 0.0, 0.0, 0.0]
        target, applied = bounded_increment_target(current, 1, -20.0)
        self.assertAlmostEqual(applied, -6.0, places=5)
        self.assertAlmostEqual(target[1], 0.0, places=6)

    def test_increment_at_boundary_does_not_move(self):
        current = [math.radians(150.0), 0.5, -0.5, 0.0, 0.0, 0.0]
        target, applied = bounded_increment_target(current, 0, 20.0)
        self.assertAlmostEqual(applied, 0.0)
        self.assertEqual(target, current)

    def test_quaternion_to_euler_identity(self):
        roll, pitch, yaw = quaternion_to_euler_degrees(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_quaternion_to_euler_yaw(self):
        half = math.radians(90.0) / 2.0
        _, _, yaw = quaternion_to_euler_degrees(
            0.0, 0.0, math.sin(half), math.cos(half)
        )
        self.assertAlmostEqual(yaw, 90.0)

    def test_simulation_uses_six_joint_chain(self):
        transforms = link_transforms([0.0] * 6)
        self.assertEqual(len(transforms), 7)
        self.assertAlmostEqual(transforms[1][2, 3], 0.123)

    def test_simulation_rejects_incomplete_joint_data(self):
        with self.assertRaises(ValueError):
            link_transforms([0.0] * 5)

    def test_accepts_gripper_limits(self):
        self.assertTrue(validate_gripper_target(0.0, 0.5)[0])
        self.assertTrue(validate_gripper_target(0.1, 3.0)[0])

    def test_rejects_invalid_gripper_target(self):
        self.assertFalse(validate_gripper_target(-0.001, 1.0)[0])
        self.assertFalse(validate_gripper_target(0.05, 3.1)[0])

    def test_gripper_fingers_follow_reported_width(self):
        _, base, finger1, finger2 = gripper_transforms(
            link_transforms([0.0] * 6)[-1], 0.1
        )
        base_inverse = np.linalg.inv(base)
        finger1_in_base = base_inverse @ finger1
        finger2_in_base = base_inverse @ finger2
        distance = abs(finger1_in_base[1, 3] - finger2_in_base[1, 3])
        self.assertAlmostEqual(distance, 0.1, places=6)


if __name__ == "__main__":
    unittest.main()
