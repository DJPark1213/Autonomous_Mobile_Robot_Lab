"""Task 3 TODO: implement the assignment's cruise PID equation."""


class PidSpeedController:
    """Placeholder for speed control using odometry feedback."""

    def __init__(self, config):
        self.config = config

    def reset(self):
        """TODO: reset controller history."""
        pass

    def update(self, setpoint, measurement, dt):
        """TODO: calculate the speed command; return zero for now."""
        return 0.0
