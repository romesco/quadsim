"""Unit tests for the motor model class."""
from absl.testing import parameterized
from ml_collections import config_dict
import numpy as np
import unittest

from quadsim.robots.motor_model import MotorModel, MotorControlMode


class TestMotorModel(parameterized.TestCase):
    """Test the motor model class."""

    def setUp(self):
        self.motor_model = MotorModel(
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

    @parameterized.parameters(
        [
            (20.0, 20.0, 20.0),
            (50, 40, 50),
            (-20, -20, -20),
            (-70, -60, -70),
        ]
    )
    def test_torque_control(
        self, motor_command, expected_applied_torque, expected_observed_torque
    ):
        current_angle = 0
        current_velocity = 0
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            (0.0, 0.0, 0.2, 20.0, 20.0),
            (0.0, 1.0, 0.2, 19.0, 19.0),
            (1.0, 1.0, 1.2, 19.0, 19.0),
            (0.0, 0.0, 2.0, 40.0, 200.0),
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
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            (0.0, 0.0, 0.2, 5.0, 5.0, 5.0),
            (0.0, 1.0, 0.2, 0.0, 0.0, 0.0),
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
        command = np.zeros(5)
        command[0] = desired_angle
        command[4] = desired_extra_torque
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            command, current_angle, current_velocity, MotorControlMode.HYBRID
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            (0.0, 0.0, 0.2, 5.0, 25.0, 25.0),
            (0.0, 1.0, 0.2, -1.0, 18.0, 18.0),
            (1.0, 1.0, 1.2, 0.0, 19.0, 19.0),
            (0.0, 0.0, 2.0, 10.0, 40.0, 210.0),
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
        command = np.zeros(5)
        command[[0]] = desired_angle
        command[[1]] = 100
        command[[3]] = 1
        command[[4]] = desired_extra_torque
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            command, current_angle, current_velocity, MotorControlMode.HYBRID
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [
            (20.0, 1.0, 20.0, 20.0),
            (20.0, 0.5, 10.0, 20.0),
            (60.0, 0.8, 32.0, 60.0),
            (40.0, 1.0, 40.0, 40.0),
        ]
    )
    def test_set_strength_ratios(
        self,
        motor_command,
        strength_ratios,
        expected_applied_torque,
        expected_observed_torque,
    ):
        self.motor_model.set_strength_ratio(strength_ratios)
        current_angle = 0.0
        current_velocity = 0.0
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)

    @parameterized.parameters(
        [(100.0, 1.0, 0.2, 20.0, 20.0), (1.0, 1.0, 0.2, 0.2, 0.2)]
    )
    def test_set_pd_gain(
        self, kp, kd, desired_angle, expected_applied_torque, expected_observed_torque
    ):
        self.motor_model.set_motor_gain(kp, kd)
        current_angle = 0.0
        current_velocity = 0.0
        applied_torque, observed_torque = self.motor_model.convert_to_torque(
            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
        )
        np.testing.assert_allclose(applied_torque, expected_applied_torque)
        np.testing.assert_allclose(observed_torque, expected_observed_torque)


if __name__ == "__main__":
    unittest.main()
