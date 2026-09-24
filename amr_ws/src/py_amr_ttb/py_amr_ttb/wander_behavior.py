"""Generate autonomous-wandering velocity commands."""

import random
from enum import Enum

from geometry_msgs.msg import Twist


class WanderState(Enum):
    """Internal phases of the obstacle-avoidance behavior."""

    DRIVE = 'drive'
    TURN = 'turn'


class WanderBehavior:
    """Drive, explicitly stop at an obstacle, turn, and then resume.

    This class only returns commands. The combined Lab 1 controller must be
    the single node that publishes the final command to `/TTB10/cmd_vel`.
    """

    def __init__(self, config):
        self.config = config
        self.state = WanderState.DRIVE
        self.turn_direction = 1.0
        self.turn_end_time = 0.0

    def reset(self):
        """Return to the initial state when WANDER mode is selected."""
        self.state = WanderState.DRIVE
        self.turn_direction = 1.0
        self.turn_end_time = 0.0

    def command(self, now, ir_state):
        """Return this control tick's ``Twist`` and an optional error.

        A zero ``Twist`` is deliberately returned for one tick whenever a
        transition begins or ends. That makes the required stop visible and
        prevents forward and turning commands from being mixed accidentally.
        """
        if not ir_state.is_fresh(now):
            return Twist(), 'IR data is missing/stale.'

        validation_error = ir_state.validation_error()
        if validation_error is not None:
            return Twist(), validation_error

        obstacle = ir_state.obstacle_detected()
        if self.state is WanderState.DRIVE:
            if obstacle:
                self._begin_random_turn(now)
                # Required explicit stop before the first turning command.
                return Twist(), None

            output = Twist()
            output.linear.x = self.config.WANDER_FORWARD_SPEED
            return output, None

        if now >= self.turn_end_time:
            if obstacle:
                self._begin_random_turn(now)
                # Stop briefly before trying a new random turn.
                return Twist(), None

            self.state = WanderState.DRIVE
            # Stop at the TURN -> DRIVE boundary; forward motion starts on
            # the next control tick after the path has been checked again.
            return Twist(), None

        output = Twist()
        output.angular.z = (
            self.turn_direction * self.config.WANDER_TURN_SPEED
        )
        return output, None

    def _begin_random_turn(self, now):
        """Choose a left/right timed turn using tomorrow's tunable values."""
        self.state = WanderState.TURN
        self.turn_direction = random.choice((-1.0, 1.0))
        duration = random.uniform(
            self.config.WANDER_TURN_MIN_TIME,
            self.config.WANDER_TURN_MAX_TIME,
        )
        self.turn_end_time = float(now) + duration
