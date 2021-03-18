"""Base class for all robots."""


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

    def apply_action(self, action, motor_control_mode=None):
        pass
