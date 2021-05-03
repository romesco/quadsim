"""Unit tests for the motor model class."""
from typing import Tuple

import numpy as np
import pytest
from quadsim.robots.motors import MotorControlMode
from quadsim.robots.motors import MotorGroup
from quadsim.robots.motors import MotorModel


@pytest.fixture
def motors():
    return MotorGroup(
        [
            MotorModel(
                motor_control_mode=MotorControlMode.POSITION,
                min_position=-3,
                max_position=2,
                min_velocity=-100,
                max_velocity=100,
                min_torque=-60,
                max_torque=40,
                kp=100,
                kd=1,
            ),
            MotorModel(
                motor_control_mode=MotorControlMode.POSITION,
                min_position=-2,
                max_position=3,
                min_velocity=-100,
                max_velocity=100,
                min_torque=-50,
                max_torque=50,
                kp=100,
                kd=1,
            ),
        ]
    )


@pytest.mark.parametrize(
    "motor_command, expected_applied_torque, expected_observed_torque",
    [
        ((20.0, 30.0), (20.0, 30.0), (20.0, 30.0)),
        ((50.0, 30.0), (40.0, 30.0), (50.0, 30.0)),
        ((-20.0, 60.0), (-20.0, 50.0), (-20.0, 60.0)),
        ((-70.0, -60.0), (-60.0, -50.0), (-70.0, -60.0)),
    ],
)
def test_torque_control(
    motors,
    motor_command: Tuple[float, float],
    expected_applied_torque: Tuple[float, float],
    expected_observed_torque: Tuple[float, float],
) -> None:

    # instantiate MotorGroup (containing 2 MotorModels)
    current_angle = np.zeros(2)
    current_velocity = np.zeros(2)
    applied_torque, observed_torque = motors.convert_to_torque(
        motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)


# TODO(yxyang) Complete testing for MotorGroup / Motor functionality.
# We have partially implemented these tests, but there are still some commented lines to be converted.

@pytest.mark.parametrize(
    "current_angle, current_velocity, desired_angle, expected_applied_torque, "
    "expected_observed_torque",
    [
        ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
        ((0.0, 0.0), (1, -1), (0.2, 0.1), (19.0, 11.0), (19.0, 11.0)),
        ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (19.0, 11.0), (19.0, 11.0)),
        ((0.0, 0.0), (0.0, 0.0), (2, 1), (40.0, 50.0), (200.0, 100.0)),
    ]
)
def test_position_control(
    motors,
    current_angle,
    current_velocity,
    desired_angle,
    expected_applied_torque,
    expected_observed_torque,
):
    applied_torque, observed_torque = motors.convert_to_torque(
        desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)


@pytest.mark.parametrize(
    "current_angle, current_velocity, desired_angle, desired_extra_torque, "
    "expected_applied_torque, expected_observed_torque",
    [
        ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (5.0, 10.0), (5.0, 10.0)),
        ((0.0, 0.0), (1, -1), (0.2, 0.1), (0, 0), (0.0, 0), (0.0, 0.0)),
    ]
)
def test_hybrid_control_no_pd(
    motors,
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
    applied_torque, observed_torque = motors.convert_to_torque(
        command, current_angle, current_velocity, MotorControlMode.HYBRID
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)


@pytest.mark.parametrize(
    "current_angle, current_velocity, desired_angle, desired_extra_torque, "
    "expected_applied_torque, expected_observed_torque",
    [
        ((0.0, 0.0), (0.0, 0.0), (0.2, 0.1), (5, 10), (25.0, 20.0), (25.0, 20.0)),
        ((0.0, 0.0), (1, -1), (0.2, 0.1), (-1, -2), (18.0, 9.0), (18.0, 9.0)),
        ((1.0, 1.0), (1.0, -1.0), (1.2, 1.1), (0, 0), (19.0, 11.0), (19.0, 11.0)),
        ((0.0, 0.0), (0.0, 0.0), (2, 1), (10, 10), (40.0, 50.0), (210.0, 110.0)),
    ]
)
def test_hybrid_control_with_pd(
    motors,
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
    applied_torque, observed_torque = motors.convert_to_torque(
        command, current_angle, current_velocity, MotorControlMode.HYBRID
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)


@pytest.mark.parametrize(
    "motor_command, strength_ratios, expected_applied_torque, "
    "expected_observed_torque",
     [
         ((20.0, 30.0), 1.0, (20.0, 30.0), (20.0, 30.0)),
         ((20.0, 30.0), 0.5, (10.0, 15.0), (20.0, 30.0)),
         ((60.0, 30.0), 0.8, (32.0, 24.0), (60.0, 30.0)),
         ((40.0, 60.0), (1.0, 0.8), (40.0, 40.0), (40.0, 60.0)),
     ]
 )
def test_set_strength_ratios(
    motors,
    motor_command,
    strength_ratios,
    expected_applied_torque,
    expected_observed_torque,
):
    motors.strength_ratios = strength_ratios
    current_angle = np.zeros(2)
    current_velocity = np.zeros(2)
    applied_torque, observed_torque = motors.convert_to_torque(
        motor_command, current_angle, current_velocity, MotorControlMode.TORQUE
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)


@pytest.mark.parametrize(
    "kp, kd, desired_angle, expected_applied_torque, expected_observed_torque",
    [
        ((100.0, 100.0), (1.0, 1.0), (0.2, 0.1), (20.0, 9.0), (20.0, 9.0)),
        ((100.0, 100.0), (1.0, 0.0), (0.2, 0.1), (20.0, 10.0), (20.0, 10.0)),
        ((1.0, 100.0), (1.0, 1.0), (0.2, 0.1), (0.2, 9.0), (0.2, 9.0)),
    ]
)
def test_set_pd_gain(
    motors, kp, kd, desired_angle, expected_applied_torque, expected_observed_torque
):
    motors.kps = kp
    motors.kds = kd
    current_angle = np.zeros(2)
    current_velocity = np.array([0.0, 1.0])
    applied_torque, observed_torque = motors.convert_to_torque(
        desired_angle, current_angle, current_velocity, MotorControlMode.POSITION
    )
    np.testing.assert_allclose(applied_torque, expected_applied_torque)
    np.testing.assert_allclose(observed_torque, expected_observed_torque)
