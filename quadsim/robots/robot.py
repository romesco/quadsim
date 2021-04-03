"""Base class for all robots."""
from absl import logging

import numpy as np
import pybullet_data

from quadsim.robots import motor_model


class Robot:
    """Robot Base."""

    def __init__(self, pybullet_client, config) -> None:
        """Constructs a base robot and resets it to the initial states.

        Args:
          pybullet_client: The instance of BulletClient to manage different
            simulations.
          motors: A list of motor models.
          sensors: A list of sensor models.
          on_rack: Whether the robot is hanging in mid-air or not. Used for debugging.
        """
        self._pybullet_client = pybullet_client
        self.config = config
        self._setup_simulator()
        self._load_robot_urdf()
        self._motor_model = motor_model.MotorModel(self.config.motor)
        self._num_motors = self._motor_model.num_motors
        self._motor_torques = None
        self._step_counter = 0
        self.reset()

    def _setup_simulator(self) -> None:
        """Sets up the pybullet simulator based on robot config."""
        p = self._pybullet_client
        sim_config = self.config.simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setPhysicsEngineParameter(
            numSolverIterations=sim_config.num_solver_iterations
        )
        p.setTimeStep(sim_config.timestep)
        p.setGravity(0.0, 0.0, -9.8)

    def _load_robot_urdf(self) -> None:
        p = self._pybullet_client

        if self.config.on_rack:
            self.quadruped = p.loadURDF(
                self.config.urdf_path, self.config.init_rack_position
            )
            self.rack_constraint = p.createConstraint(
                parentBodyUniqueId=self.quadruped,
                parentLinkIndex=-1,
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=self._pybullet_client.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=self.config.init_rack_position,
                childFrameOrientation=[0.0, 0.0, 0.0, 1],
            )
        else:
            self.quadruped = p.loadURDF(
                self.config.urdf_path, self.config.init_position
            )

        self._build_urdf_ids()

    def _build_urdf_ids(self) -> None:
        """Records ids of base link, foot links and motor joints.

        For detailed documentation of links and joints, please refer to the
        pybullet documentation at:
        https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.e27vav9dy7v6
        """
        self._chassis_link_ids = [-1]
        self._motor_joint_ids = []
        self._foot_link_ids = []

        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        for joint_id in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, joint_id)
            joint_name = joint_info[1].decode("UTF-8")
            if joint_name in self.config.base_joint_names:
                self._chassis_link_ids.append(joint_id)
            elif joint_name in self.config.motor_joint_names:
                self._motor_joint_ids.append(joint_id)
            elif joint_name in self.config.foot_joint_names:
                self._foot_link_ids.append(joint_id)

    def reset(self, hard_reset: bool = False) -> None:
        """Resets the robot."""
        if hard_reset:
            # This assumes that resetSimulation() is already called.
            self._load_robot_urdf()
        else:
            init_position = (
                self.config.init_rack_position
                if self.config.on_rack
                else self.config.init_position
            )
            self._pybullet_client.resetBasePositionAndOrientation(
                self.quadruped, init_position, [0.0, 0.0, 0.0, 1.0]
            )

        # Remove velocity and force constraint on all joints
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
        for i in range(len(self._motor_joint_ids)):
            self._pybullet_client.resetJointState(
                self.quadruped,
                self._motor_joint_ids[i],
                self.config.init_motor_angles[i],
                targetVelocity=0,
            )

        # Steps the robot with position command
        num_reset_steps = int(
            self.config.simulation.reset_time / self.config.simulation.timestep
        )
        for _ in range(num_reset_steps):
            self.step(
                self.config.init_motor_angles, motor_model.MotorControlMode.POSITION
            )
        self._step_counter = 0

    def _apply_action(self, action, motor_control_mode=None) -> None:
        torques, observed_torques = self._motor_model.convert_to_torque(
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
        for _ in range(self.config.simulation.action_repeat):
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
        return self.config.simulation.timestep * self.config.simulation.action_repeat

    @property
    def time_since_reset(self):
        return self._step_counter * self.control_timestep
