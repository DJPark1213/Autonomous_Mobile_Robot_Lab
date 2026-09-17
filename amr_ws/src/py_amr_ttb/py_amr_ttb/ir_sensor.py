"""Task 2 TODO: interpret IR readings and calibrate obstacle detection."""


class IrSensorState:
    """Placeholder for IR sensor state."""

    def __init__(self, config):
        self.config = config

    def update(self, message, timestamp):
        """TODO: read and validate the robot's IR message."""
        return 'Task 2: IR processing is not implemented.'

    def is_fresh(self, now):
        """TODO: check the age of the last valid reading."""
        return False

    def obstacle_detected(self):
        """TODO: apply a calibrated obstacle rule; default to blocked."""
        return True
