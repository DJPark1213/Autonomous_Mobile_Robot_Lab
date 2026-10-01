"""Generate autonomous wandering commands with stable turns."""

import random
from enum import Enum

from geometry_msgs.msg import Twist


class WanderState(Enum):
    """Phases of autonomous wandering."""

    DRIVE = 'drive'
    TURN = 'turn'


class WanderBehavior:
    """Stop once, turn consistently, and resume when the path is clear."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Reset the behavior when wandering is selected."""
        self.state = WanderState.DRIVE
        self.turn_direction = 1.0
        self.turn_end_time = 0.0
        self.clear_count = 0
        self.last_ir_time = None

    def command(self, now, ir_state):
        """Return one velocity command and an optional warning."""
        if not ir_state.is_fresh(now):
            self.clear_count = 0
            self.last_ir_time = None
            return Twist(), 'IR data is missing/stale.'

        error = ir_state.validation_error()
        if error is not None:
            self.clear_count = 0
            self.last_ir_time = None
            return Twist(), error

        if self.state is WanderState.DRIVE:
            if ir_state.obstacle_detected():
                self._begin_random_turn(now)

                # Stop for one control tick before turning.
                return Twist(), None

            return self._drive_command(), None

        # Count new sensor messages, not repeated controller ticks.
        if ir_state.last_message_time != self.last_ir_time:
            self.last_ir_time = ir_state.last_message_time

            if self._path_is_clear(ir_state):
                self.clear_count = min(
                    self.clear_count + 1,
                    self.config.WANDER_CLEAR_SAMPLES,
                )
            else:
                self.clear_count = 0

        if (
            now >= self.turn_end_time
            and self.clear_count >= self.config.WANDER_CLEAR_SAMPLES
        ):
            self.state = WanderState.DRIVE
            return self._drive_command(), None

        # Keep the same direction until the path is clear.
        output = Twist()
        output.angular.z = (
            self.turn_direction * self.config.WANDER_TURN_SPEED
        )
        return output, None

    def _path_is_clear(self, ir_state):
        """Use a separate threshold before allowing forward motion."""
        indices = self.config.IR_FRONT_SENSOR_INDICES
        values = ir_state.values

        if indices is not None:
            values = [values[index] for index in indices]

        threshold = self.config.IR_CLEAR_THRESHOLD

        if self.config.IR_OBSTACLE_WHEN_ABOVE_THRESHOLD:
            return all(value <= threshold for value in values)

        return all(value >= threshold for value in values)

    def _drive_command(self):
        """Keep the assignment's constant forward command."""
        output = Twist()
        output.linear.x = self.config.WANDER_FORWARD_SPEED
        return output

    def _begin_random_turn(self, now):
        """Choose a random direction and minimum turning duration."""
        self.state = WanderState.TURN
        self.turn_direction = random.choice((-1.0, 1.0))

        duration = random.uniform(
            self.config.WANDER_TURN_MIN_TIME,
            self.config.WANDER_TURN_MAX_TIME,
        )

        self.turn_end_time = float(now) + duration
        self.clear_count = 0
        self.last_ir_time = None
