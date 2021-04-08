"""Unit tests for the motor model class."""
from absl.testing import parameterized
import numpy as np
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
import unittest

from quadsim.configs.robots import config_a1
from quadsim.robots.robot import Robot


class TestMotorModel(parameterized.TestCase):
    """Test the motor model class."""

    @classmethod
    def setUpClass(cls):
        p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        cls.pybullet_client = p

    def setUp(self):
        self.pybullet_client.resetSimulation()
        self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.pybullet_client.loadURDF("plane.urdf")

    @parameterized.parameters(
        [(True, True), (True, False), (False, True), (False, False)]
    )
    def test_reset(self, on_rack, hard_reset):
        config = config_a1.get_config()
        config.on_rack = on_rack
        robot = Robot(self.pybullet_client, config)
        if hard_reset:
            self.pybullet_client.resetSimulation()
            self.pybullet_client.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.pybullet_client.loadURDF("plane.urdf")
        robot.reset(hard_reset=hard_reset)
        if on_rack:
            np.testing.assert_allclose(
                robot.base_position, config.init_rack_position, atol=0.01
            )
        else:
            np.testing.assert_allclose(
                robot.base_position, config.init_position, atol=0.1
            )
        np.testing.assert_allclose(
            robot.base_orientation_rpy, [0.0, 0.0, 0.0], atol=0.03
        )
        np.testing.assert_allclose(
            robot.base_orientation_quat, [0.0, 0.0, 0.0, 1.0], atol=0.01
        )
        np.testing.assert_allclose(robot.base_velocity, [0.0, 0.0, 0.0], atol=0.01)
        np.testing.assert_allclose(robot.base_rpy_rate, [0.0, 0.0, 0.0], atol=0.01)
        if on_rack:
            np.testing.assert_allclose(
                robot.motor_angles, config.init_motor_angles, atol=0.01
            )
        else:
            np.testing.assert_allclose(
                robot.motor_angles, config.init_motor_angles, atol=0.1
            )
        np.testing.assert_allclose(robot.motor_velocities, np.zeros(12), atol=0.01)
        np.testing.assert_allclose(
            robot.control_timestep,
            config.simulation.timestep * config.simulation.action_repeat,
        )
        np.testing.assert_allclose(robot.time_since_reset, 0.0)

    def test_step(self):
        config = config_a1.get_config()
        robot = Robot(self.pybullet_client, config)
        for _ in range(10):
            robot.step(config.init_motor_angles)
        np.testing.assert_equal(robot.time_since_reset, 10 * robot.control_timestep)


if __name__ == "__main__":
    unittest.main()
