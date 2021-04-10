from quadsim.robots.motors import MotorControlMode, MotorModel, MotorGroup
from quadsim.robots.robot import Robot
from quadsim.simulator import SimulatorConf

import pybullet
from pybullet_utils import bullet_client

if __name__ == '__main__':

    sim_conf = SimulatorConf(
        show_gui=False,
        on_rack=True,
    )

    # Set up pybullet client
    if sim_conf.show_gui:
        p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
    else:
        p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)

    motor1 = MotorModel(
        name="FR_hip_joint",
        motor_control_mode=MotorControlMode.POSITION,
        init_position=0.0,
        min_position=-0.802851455917,
        max_position=0.802851455917,
        min_velocity=-16,
        max_velocity=16,
        min_torque=-33.5,
        max_torque=33.5,
        kp=100,
        kd=1,
    )

    motor2 = MotorModel(
        name="FR_upper_joint",
        motor_control_mode=MotorControlMode.POSITION,
        init_position=0.0,
        min_position=-0.802851455917,
        max_position=0.802851455917,
        min_velocity=-16,
        max_velocity=16,
        min_torque=-33.5,
        max_torque=33.5,
        kp=100,
        kd=1,
    )

    motor_group = MotorGroup(
        motors=[
            motor1,
            motor2,
        ]
    )

    robot = Robot(
        pybullet_client=p,
        sim_conf=sim_conf,
        urdf_path='a1/a1.urdf',
        motors=motor_group,
        base_joint_names=[],
        foot_joint_names=[
            "FR_toe_fixed",
            "FL_toe_fixed",
            "RR_toe_fixed",
            "RL_toe_fixed",
        ],
    )

    # Print out all currently set instance attributes
    for attr, val in motor_group.__dict__.items():
        print(attr, '=', val)
        if attr == '_motors':
            for motor_num, motor in enumerate(val):
                print(f'===Motor {motor_num+1}:')
                for bttr, vbl in motor.__dict__.items():
                    print(bttr, '=', vbl)
            print('===MotorGroup:')
