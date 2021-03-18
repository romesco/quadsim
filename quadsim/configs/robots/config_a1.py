"""Robot configurations for the a1 robot."""
import ml_collections
import numpy as np

from qudsim.robots.motor_model import MotorControlMode


def get_config():
    config = ml_collections.ConfigDict()

    # Simulation configs
    sim_config = ml_collections.ConfigDict()
    sim_config.timestep = 0.002
    sim_config.action_repeat = 1
    sim_config.hard_reset = True
    sim_config.on_rack = False
    sim_config.init_position = [0.0, 0.0, 0.32]
    sim_config.init_rack_position = [0.0, 0.0, 1]
    sim_config.reset_time = 3
    config.simulation = sim_config

    # urdf and kinematics
    config.urdf_path = "a1/a1.urdf"
    config.base_link_names = ["trunk"]
    config.motor_joint_names = [
        "FR_hip_joint",
        "FR_upper_joint",
        "FR_lower_joint",
        "FL_hip_joint",
        "FL_upper_joint",
        "FL_lower_joint",
        "RR_hip_joint",
        "RR_upper_joint",
        "RR_lower_joint",
        "RL_hip_joint",
        "RL_upper_joint",
        "RL_lower_joint",
    ]
    config.foot_link_names = ["FR_toe", "FL_toe", "RR_toe", "RL_toe"]
    config.init_motor_angles = np.array([0.0, 0.9, -1.8] * 4)

    # Motor control configs
    motor_config = ml_collections.ConfigDict()
    motor_config.num_motors = 12
    motor_config.motor_control_mode = MotorControlMode.POSITION
    motor_config.max_position = np.array(
        [0.802851455917, 4.18879020479, -0.916297857297] * 4
    )
    motor_config.min_position = np.array(
        [-0.802851455917, -1.0471975512, -2.69653369433] * 4
    )
    motor_config.max_torque = np.ones(12) * 33.5
    motor_config.min_torque = -np.ones(12) * 33.5
    motor_config.max_velocity = np.ones(12) * 16
    motor_config.min_velocity = -np.ones(12) * 16
    motor_config.kps = np.ones(12) * 100
    motor_config.kds = np.array([1.0, 2.0, 2.0] * 4)
    config.motor = motor_config

    return config
