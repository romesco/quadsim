"""Unit tests for the motor model class."""
from absl.testing import parameterized
from ml_collections import config_dict
import numpy as np
import unittest

from quadsim.robots.motor_group import MotorGroup, MotorControlMode


def get_default_config():
    return [
        config_dict.ConfigDict(
            dict(
                joint_name="joint_1",
                motor_control_mode=MotorControlMode.POSITION,
                kp=100,
                kd=1,
                max_torque=40,
                min_torque=-60,
                max_velocity=100,
                min_velocity=-100,
                max_position=2,
                min_position=-3,
            )
        ),
        config_dict.ConfigDict(
            dict(
                joint_name="joint_2",
                motor_control_mode=MotorControlMode.POSITION,
                kp=100,
                kd=1,
                max_torque=50,
                min_torque=-50,
                max_velocity=100,
                min_velocity=-100,
                max_position=3,
                min_position=-2,
            )
        ),
    ]


class TestMotorGroup(parameterized.TestCase):
    """Test the motor model class."""

    def setUp(self):
        config = get_default_config()
        self.motor_group = MotorGroup(config)

    @parameterized.parameters(
        [
            ((20.0, 30.0), (20.0, 30.0), (20.0, 30.0)),
            ((50.0, 30.0), (40.0, 30.0), (50.0, 30.0)),
            ((-20.0, 60.0), (-20.0, 50.0), (-20.0, 60.0)),
            ((-70.0, -60.0), (-60.0, -50.0), (-70.0, -60.0)),
        ]
    )
    def test_torque_control(
        self, motor_command, expected_applied_torque, expected_observed_torque
    ):
        current_angle = np.zeros(2)
        current_velocity = np.zeros(2)
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
            ((0.0, 0.0), (1, -1), (0.2, 0.1), (19.0, 11.0), (19.0, 11.0)),
            ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (19.0, 11.0), (19.0, 11.0)),
            ((0.0, 0.0), (0.0, 0.0), (2, 1), (40.0, 50.0), (200.0, 100.0)),
        ]
    )
    def test_position_control(
        self,
        current_angle,
        current_velocity,
        desired_angle,
        expected_applied_torque,
        expected_observed_torque,
    ):
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (5.0, 10.0), (5.0, 10.0)),
            ((0.0, 0.0), (1, -1), (0.2, 0.1), (0, 0), (0.0, 0), (0.0, 0.0)),
        ]
    )
    def test_hybrid_control_no_pd(
        self,
        current_angle,
        current_velocity,
        desired_angle,
        desired_extra_torque,
        expected_applied_torque,
        expected_observed_torque,
    ):
        command = np.zeros(10)
        command[[0, 5]] = desired_angle
        command[[4, 9]] = desired_extra_torque
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            command, current_angle, current_velocity, MotorControlMode.HYBRID
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (25.0, 20.0), (25.0, 20.0)),
            ((0.0, 0.0), (1, -1), (0.2, 0.1), (-1, -2), (18.0, 9.0), (18.0, 9.0)),
            ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (0, 0), (19.0, 11.0), (19.0, 11.0)),
            ((0.0, 0.0), (0.0, 0.0), (2, 1), (10, 10), (40.0, 50.0), (210.0, 110.0)),
        ]
    )
    def test_hybrid_control_with_pd(
        self,
        current_angle,
        current_velocity,
        desired_angle,
        desired_extra_torque,
        expected_applied_torque,
        expected_observed_torque,
    ):
        command = np.zeros(10)
        command[[0, 5]] = desired_angle
        command[[1, 6]] = 100
        command[[3, 8]] = 1
        command[[4, 9]] = desired_extra_torque
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            command, current_angle, current_velocity, MotorControlMode.HYBRID
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            ((20.0, 30.0), 1.0, (20.0, 30.0), (20.0, 30.0)),
            ((20.0, 30.0), 0.5, (10.0, 15.0), (20.0, 30.0)),
            ((60.0, 30.0), 0.8, (32.0, 24.0), (60.0, 30.0)),
            ((40.0, 60.0), (1.0, 0.8), (40.0, 40.0), (40.0, 60.0)),
        ]
    )
    def test_set_strength_ratios(
        self,
        motor_command,
        strength_ratios,
        expected_applied_torque,
        expected_observed_torque,
    ):
        self.motor_group.strength_ratios = strength_ratios
        current_angle = np.zeros(2)
        current_velocity = np.zeros(2)
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            ((100.0, 100.0), (1.0, 1.0), (0.2, 0.1), (20.0, 9.0), (20.0, 9.0)),
            ((100.0, 100.0), (1.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
            ((1.0, 100.0), (1.0, 1.0), (0.2, 0.1), (0.2, 9.0), (0.2, 9.0)),
        ]
    )
    def test_set_pd_gain(
        self, kp, kd, desired_angle, expected_applied_torque, expected_observed_torque
    ):
        self.motor_group.kp = kp
        self.motor_group.kd = kd
        current_angle = np.zeros(2)
        current_velocity = np.array([0.0, 1.0])
        applied_torque, observed_torque = self.motor_group.convert_to_torque(
            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)


if __name__ == "__main__":
    unittest.main()
