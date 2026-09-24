"""Speed PID using the accumulated-output equation from the lab."""

import math


class PidSpeedController:
    """Calculate velocity commands without publishing ROS messages."""

    def __init__(self, config):
        self.config = config
        self.min_output = 0.0
        self.max_output = 0.4
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.output = 0.0

    def update(self, setpoint, measurement, dt):
        if (
            not all(
                math.isfinite(x)
                for x in (setpoint, measurement, dt)
            )
            or dt <= 0.0
            or setpoint <= 0.0
        ):
            self.reset()
            return 0.0

        error = setpoint - measurement

        self.integral += error * dt

        derivative = (
            error - self.previous_error
        ) / dt

        self.output += (
            self.config.KP * error
            + self.config.KI * self.integral
            + self.config.KD * derivative
        )

        self.output = max(
            self.min_output,
            min(self.max_output, self.output)
        )

        self.previous_error = error

        return self.output
