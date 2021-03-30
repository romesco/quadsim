"""Unit tests for the motor model class."""
import pytest
import numpy as np
from typing import Tuple

from quadsim.robots.motor_model import MotorControlMode

from hydra.utils import instantiate
from hydra_configs.quadsim.robots.motor_model import MotorModelConf

motor_model_cfg = MotorModelConf(
    motor_control_mode=MotorControlMode.POSITION,
    num_motors=2,
    kps=[100, 100],
    kds=[1, 1],
    max_torque=[40, 50],
    min_torque=[-60, -50],
    max_velocity=[100, 100],
    min_velocity=[-100, -100],
    max_position=[2, 3],
    min_position=[-3, -2],
)


@pytest.mark.parametrize(
    "motor_command, expected_applied_torque, expected_observed_torque",
    [
        pytest.param(
            (20.0, 30.0),
            (20.0, 30.0),
            (20.0, 30.0),
            id="0",
        ),
        pytest.param(
            (50.0, 30.0),
            (40.0, 30.0),
            (50.0, 30.0),
            id="1",
        ),
        pytest.param(
            (-20.0, 60.0),
            (-20.0, 50.0),
            (-20.0, 60.0),
            id="2",
        ),
        pytest.param(
            (-70.0, -60.0),
            (-60.0, -50.0),
            (-70.0, -60.0),
            id="3",
        ),
    ]
)
def test_torque_control(
    motor_command: Tuple[float, float],
    expected_applied_torque: Tuple[float, float],
    expected_observed_torque: Tuple[float, float],
) -> None:
    motor_model = instantiate(motor_model_cfg)
    current_angle = np.zeros(2)
    current_velocity = np.zeros(2)
    applied_torque, observed_torque = motor_model.convert_to_torque(
        motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)

#    @parameterized.parameters(
#        [
#            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
#            ((0.0, 0.0), (1, -1), (0.2, 0.1), (19.0, 11.0), (19.0, 11.0)),
#            ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (19.0, 11.0), (19.0, 11.0)),
#            ((0.0, 0.0), (0.0, 0.0), (2, 1), (40.0, 50.0), (200.0, 100.0)),
#        ]
#    )
#    def test_position_control(
#        self,
#        current_angle,
#        current_velocity,
#        desired_angle,
#        expected_applied_torque,
#        expected_observed_torque,
#    ):
#        applied_torque, observed_torque = self.motor_model.convert_to_torque(
#            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
#        )
#        np.testing.assert_allclose(applied_torque, expected_applied_torque)
#        np.testing.assert_allclose(observed_torque, expected_observed_torque)
#
#    @parameterized.parameters(
#        [
#            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (5.0, 10.0), (5.0, 10.0)),
#            ((0.0, 0.0), (1, -1), (0.2, 0.1), (0, 0), (0.0, 0), (0.0, 0.0)),
#        ]
#    )
#    def test_hybrid_control_no_pd(
#        self,
#        current_angle,
#        current_velocity,
#        desired_angle,
#        desired_extra_torque,
#        expected_applied_torque,
#        expected_observed_torque,
#    ):
#        command = np.zeros(10)
#        command[[0, 5]] = desired_angle
#        command[[4, 9]] = desired_extra_torque
#        applied_torque, observed_torque = self.motor_model.convert_to_torque(
#            command, current_angle, current_velocity, MotorControlMode.HYBRID
#        )
#        np.testing.assert_allclose(applied_torque, expected_applied_torque)
#        np.testing.assert_allclose(observed_torque, expected_observed_torque)
#
#    @parameterized.parameters(
#        [
#            ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (25.0, 20.0), (25.0, 20.0)),
#            ((0.0, 0.0), (1, -1), (0.2, 0.1), (-1, -2), (18.0, 9.0), (18.0, 9.0)),
#            ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (0, 0), (19.0, 11.0), (19.0, 11.0)),
#            ((0.0, 0.0), (0.0, 0.0), (2, 1), (10, 10), (40.0, 50.0), (210.0, 110.0)),
#        ]
#    )
#    def test_hybrid_control_with_pd(
#        self,
#        current_angle,
#        current_velocity,
#        desired_angle,
#        desired_extra_torque,
#        expected_applied_torque,
#        expected_observed_torque,
#    ):
#        command = np.zeros(10)
#        command[[0, 5]] = desired_angle
#        command[[1, 6]] = 100
#        command[[3, 8]] = 1
#        command[[4, 9]] = desired_extra_torque
#        applied_torque, observed_torque = self.motor_model.convert_to_torque(
#            command, current_angle, current_velocity, MotorControlMode.HYBRID
#        )
#        np.testing.assert_allclose(applied_torque, expected_applied_torque)
#        np.testing.assert_allclose(observed_torque, expected_observed_torque)
#
#    @parameterized.parameters(
#        [
#            ((20.0, 30.0), 1.0, (20.0, 30.0), (20.0, 30.0)),
#            ((20.0, 30.0), 0.5, (10.0, 15.0), (20.0, 30.0)),
#            ((60.0, 30.0), 0.8, (32.0, 24.0), (60.0, 30.0)),
#            ((40.0, 60.0), (1.0, 0.8), (40.0, 40.0), (40.0, 60.0)),
#        ]
#    )
#    def test_set_strength_ratios(
#        self,
#        motor_command,
#        strength_ratios,
#        expected_applied_torque,
#        expected_observed_torque,
#    ):
#        self.motor_model.set_strength_ratios(strength_ratios)
#        current_angle = np.zeros(2)
#        current_velocity = np.zeros(2)
#        applied_torque, observed_torque = self.motor_model.convert_to_torque(
#            motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
#        )
#        np.testing.assert_allclose(applied_torque, expected_applied_torque)
#        np.testing.assert_allclose(observed_torque, expected_observed_torque)
#
#    @parameterized.parameters(
#        [
#            ((100.0, 100.0), (1.0, 1.0), (0.2, 0.1), (20.0, 9.0), (20.0, 9.0)),
#            ((100.0, 100.0), (1.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
#            ((1.0, 100.0), (1.0, 1.0), (0.2, 0.1), (0.2, 9.0), (0.2, 9.0)),
#        ]
#    )
#    def test_set_pd_gain(
#        self, kp, kd, desired_angle, expected_applied_torque, expected_observed_torque
#    ):
#        self.motor_model.set_motor_gains(kp, kd)
#        current_angle = np.zeros(2)
#        current_velocity = np.array([0.0, 1.0])
#        applied_torque, observed_torque = self.motor_model.convert_to_torque(
#            desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
#        )
#        np.testing.assert_allclose(applied_torque, expected_applied_torque)
#        np.testing.assert_allclose(observed_torque, expected_observed_torque)
