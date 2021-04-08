"""Robot configurations for the a1 robot."""
import ml_collections
import numpy as np

from quadsim.robots.motor_group import MotorControlMode


def get_config():
    config = ml_collections.ConfigDict()

    # Simulation configs
    sim_config = ml_collections.ConfigDict()
    sim_config.timestep = 0.002
    sim_config.action_repeat = 1
    sim_config.reset_time = 3
    sim_config.num_solver_iterations = 30
    config.simulation = sim_config

    config.init_position = [0.0, 0.0, 0.32]
    config.init_rack_position = [0.0, 0.0, 1]
    config.on_rack = False

    # urdf and kinematics
    config.urdf_path = "a1/a1.urdf"
    config.base_joint_names = []
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
    config.foot_joint_names = [
        "FR_toe_fixed",
        "FL_toe_fixed",
        "RR_toe_fixed",
        "RL_toe_fixed",
    ]
    config.init_motor_angles = np.array([0.0, 0.9, -1.8] * 4)

    config.motors = [
        ml_collections.ConfigDict(
            dict(
                joint_name="FR_hip_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=0.802851455917,
                min_position=-0.802851455917,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=1,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="FR_upper_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=4.18879020479,
                min_position=-1.0471975512,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="FR_lower_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=-0.916297857297,
                min_position=-2.6965336943,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="FL_hip_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=0.802851455917,
                min_position=-0.802851455917,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=1,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="FL_upper_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=4.18879020479,
                min_position=-1.0471975512,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="FL_lower_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=-0.916297857297,
                min_position=-2.6965336943,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RR_hip_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=0.802851455917,
                min_position=-0.802851455917,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=1,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RR_upper_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=4.18879020479,
                min_position=-1.0471975512,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RR_lower_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=-0.916297857297,
                min_position=-2.6965336943,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RL_hip_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=0.802851455917,
                min_position=-0.802851455917,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=1,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RL_upper_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=4.18879020479,
                min_position=-1.0471975512,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
        ml_collections.ConfigDict(
            dict(
                joint_name="RL_lower_joint",
                motor_control_mode=MotorControlMode.POSITION,
                max_position=-0.916297857297,
                min_position=-2.6965336943,
                max_torque=33.5,
                min_torque=-33.5,
                max_velocity=16,
                min_velocity=-16,
                kp=100,
                kd=2,
            )
        ),
    ]
    return config
