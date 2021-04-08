"""Example of running A1 robot with position control.

To run the example with gui:
python -m tests.quadsim.robots.robot_test
"""
from absl import app
from absl import flags

from ml_collections import config_flags
import numpy as np
import pybullet
import pybullet_data
from pybullet_utils import bullet_client
import time

from quadsim.robots.robot import Robot

config_flags.DEFINE_config_file(
    "robot_config", "quadsim/configs/robots/config_a1.py", "Robot config file."
)
flags.DEFINE_bool("show_gui", True, "whether to show pybullet GUI.")
FLAGS = flags.FLAGS


def get_action(t):
    mid_action = np.array([0.0, 0.9, -1.8] * 4)
    amplitude = np.array([0.0, 0.2, -0.4] * 4)
    freq = 1.0
    return mid_action + amplitude * np.sin(2 * np.pi * freq * t)


def main(argv):
    del argv  # unused

    if FLAGS.show_gui:
        p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
    else:
        p = bullet_client.BulletClient(connection_mode=pybullet.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot = Robot(p, FLAGS.robot_config)
    for _ in range(10000):
        action = get_action(robot.time_since_reset)
        robot.step(action)
        time.sleep(0.002)
        print(robot.base_orientation_rpy)


if __name__ == "__main__":
    app.run(main)
