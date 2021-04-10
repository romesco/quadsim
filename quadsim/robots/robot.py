"""Base class for all robots."""
from typing import Any, Tuple
import numpy as np

import pybullet_data

from quadsim.simulator import SimulatorConf
from quadsim.robots.motors import MotorGroup, MotorControlMode


class Robot:
    """Robot Base."""

    def __init__(
        self,
        pybullet_client: Any = None,
        sim_conf: SimulatorConf = None,
        urdf_path: str = None,
        motors: Tuple[MotorGroup, ...] = None,
        base_joint_names: Tuple[str, ...] = None,
        foot_joint_names: Tuple[str, ...] = None,
        # see simulator.py for 'rack' related config fields

    ) -> None:
        """Constructs a base robot and resets it to the initial states.
        TODO
        """
        self._sim_conf = sim_conf
        self._pybullet_client = pybullet_client
        self._motor_group = motors
        self._base_joint_names = base_joint_names
        self._foot_joint_names = foot_joint_names
        self._num_motors = self._motor_group.num_motors
        self._motor_torques = None
        self._load_robot_urdf(urdf_path)
        self._step_counter = 0
        self.reset()

    def _load_robot_urdf(self, urdf_path: str) -> None:
        p = self._pybullet_client
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        if self._sim_conf.on_rack:
            self.quadruped = p.loadURDF(
                urdf_path,
                self._sim_conf.init_rack_position
            )
            self.rack_constraint = p.createConstraint(
                parentBodyUniqueId=self.quadruped,
                parentLinkIndex=-1,
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=self._pybullet_client.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=self._sim_conf.init_rack_position,
                childFrameOrientation=[0.0, 0.0, 0.0, 1],
            )
        else:
            self.quadruped = p.loadURDF(
                urdf_path,
                self._sim_conf.init_position
            )

        self._build_urdf_ids()

    def _build_urdf_ids(self) -> None:
        """Records ids of base link, foot links and motor joints.

        For detailed documentation of links and joints, please refer to the
        pybullet documentation.
        """
        self._chassis_link_ids = [-1]
        self._motor_joint_ids = []
        self._foot_link_ids = []

        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        for joint_id in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, joint_id)
            joint_name = joint_info[1].decode("UTF-8")
            if joint_name in self._base_joint_names:
                self._chassis_link_ids.append(joint_id)
            elif joint_name in self._motor_group.motor_joint_names:
                self._motor_joint_ids.append(joint_id)
            elif joint_name in self._foot_joint_names:
                self._foot_link_ids.append(joint_id)

    def reset(self, hard_reset: bool = False) -> None:
        """Resets the robot."""
        if hard_reset:
            # This assumes that resetSimulation() is already called.
            self._load_robot_urdf()
        else:
            init_position = (
                self._sim_conf.init_rack_position
                if self._sim_conf.on_rack
                else self._sim_conf.init_position
            )
            self._pybullet_client.resetBasePositionAndOrientation(
                self.quadruped, init_position, [0.0, 0.0, 0.0, 1.0]
            )

        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        for joint_id in range(num_joints):
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(joint_id),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0,
            )

        # Set motors to the initial position
        # TODO: these should be set already when they are instantiated?
        for i in range(len(self._motor_joint_ids)):
            self._pybullet_client.resetJointState(
                self.quadruped,
                self._motor_joint_ids[i],
                self._motor_group._init_positions[i],
                targetVelocity=0,
            )

        # Steps the robot with position command
        num_reset_steps = int(
            self._sim_conf.reset_time / self._sim_conf.timestep
        )
        for _ in range(num_reset_steps):
            self.step(
                self._motor_group.init_positions, MotorControlMode.POSITION
            )
        self._step_counter = 0

    def _apply_action(self, action, motor_control_mode=None) -> None:
        torques, observed_torques = self._motor_group.convert_to_torque(
            action, self.motor_angles, self.motor_velocities, motor_control_mode
        )
        self._pybullet_client.setJointMotorControlArray(
            bodyIndex=self.quadruped,
            jointIndices=self._motor_joint_ids,
            controlMode=self._pybullet_client.TORQUE_CONTROL,
            forces=torques,
        )
        self._motor_torques = observed_torques

    def step(self, action, motor_control_mode=None) -> None:
        self._step_counter += 1
        for _ in range(self._sim_conf.action_repeat):
            self._apply_action(action, motor_control_mode)
            self._pybullet_client.stepSimulation()

    @property
    def foot_contacts(self):
        raise NotImplementedError()

    @property
    def foot_positions_in_base_frame(self):
        raise NotImplementedError()

    @property
    def foot_positions_to_motor_angles(self):
        raise NotImplementedError()

    @property
    def base_position(self):
        return np.array(
            self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[0]
        )

    @property
    def base_orientation_rpy(self):
        return self._pybullet_client.getEulerFromQuaternion(self.base_orientation_quat)

    @property
    def base_orientation_quat(self):
        return np.array(
            self._pybullet_client.getBasePositionAndOrientation(self.quadruped)[1]
        )

    @property
    def motor_angles(self):
        joint_states = self._pybullet_client.getJointStates(
            self.quadruped, self._motor_joint_ids
        )
        return np.array([s[0] for s in joint_states])

    @property
    def base_velocity(self):
        return self._pybullet_client.getBaseVelocity(self.quadruped)[0]

    @property
    def base_rpy_rate(self):
        angular_velocity = self._pybullet_client.getBaseVelocity(self.quadruped)[1]
        orientation = self.base_orientation_quat
        _, orientation_inversed = self._pybullet_client.invertTransform(
            [0, 0, 0], orientation
        )
        relative_velocity, _ = self._pybullet_client.multiplyTransforms(
            [0, 0, 0],
            orientation_inversed,
            angular_velocity,
            self._pybullet_client.getQuaternionFromEuler([0, 0, 0]),
        )
        return np.asarray(relative_velocity)

    @property
    def motor_velocities(self):
        joint_states = self._pybullet_client.getJointStates(
            self.quadruped, self._motor_joint_ids
        )
        return np.array([s[1] for s in joint_states])

    @property
    def motor_torques(self):
        return self._motor_torques

    @property
    def control_timestep(self):
        return self._sim_conf.timestep * self._sim_conf.action_repeat

    @property
    def time_since_reset(self):
        return self._step_counter * self.control_timestep
