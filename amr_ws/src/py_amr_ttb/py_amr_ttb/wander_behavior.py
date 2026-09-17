"""Task 2 TODO: implement drive, stop, random turn, and resume."""

from geometry_msgs.msg import Twist


class WanderBehavior:
    """Placeholder for autonomous wandering."""

    def __init__(self, config):
        self.config = config

    def reset(self):
        """TODO: reset the wandering state."""
        pass

    def command(self, now, ir_state):
        """TODO: select a wandering command; return zero for now."""
        return Twist(), 'Task 2: wandering is not implemented.'
