"""Cruise PID with target-speed feedforward from the supplied controller."""

import math


class PidSpeedController:
    """Compute bounded speed commands with conditional integral updates."""

    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        """Clear the accumulated error and derivative history."""
        self.integral = 0.0
        self.previous_error = None

    def update(self, setpoint, measurement, dt):
        """Return setpoint plus PID correction, or zero for stop/bad input."""
        if (
            not all(
                math.isfinite(value) for value in (setpoint, measurement, dt)
            )
            or dt <= 0.0
            or setpoint <= 0.0
        ):
            self.reset()
            return 0.0

        error = setpoint - measurement
        candidate_integral = max(
            -self.config.PID_INTEGRAL_LIMIT,
            min(self.config.PID_INTEGRAL_LIMIT, self.integral + error * dt),
        )
        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt

        # Preserve the supplied controller's equation. The assignment's
        # incremental-output equation must be checked separately with the TA.
        output = (
            setpoint
            + self.config.KP * error
            + self.config.KI * candidate_integral
            + self.config.KD * derivative
        )
        if not (
            (output > self.config.CRUISE_MAX_COMMAND and error > 0.0)
            or (output < 0.0 and error < 0.0)
        ):
            self.integral = candidate_integral

        self.previous_error = error
        return max(0.0, min(self.config.CRUISE_MAX_COMMAND, output))
