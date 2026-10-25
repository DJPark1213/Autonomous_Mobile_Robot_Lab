"""Bounded PID shared by the goal and circle controllers (no ROS dependency)."""

import math


class PID:
    """PID correction with bounded integral and optional angular derivative."""

    def __init__(self, kp, ki, kd, integral_limit=1.0, angular=False):
        """Store gains and initialize independent history."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit = integral_limit
        self.angular = angular
        self.reset()

    def reset(self):
        """Clear integral and derivative memory."""
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt, lower, upper, feedforward=0.0):
        """Return a command constrained by the caller's current limits."""
        if not all(math.isfinite(v) for v in (error, dt, lower, upper, feedforward)) or dt <= 0:
            self.reset()
            return 0.0
        delta = 0.0 if self.previous_error is None else error - self.previous_error
        if self.angular:
            delta = math.atan2(math.sin(delta), math.cos(delta))
        derivative = delta / dt
        candidate = max(-self.integral_limit,
                        min(self.integral_limit, self.integral + error * dt))
        base = feedforward + self.kp * error + self.kd * derivative
        output = base + self.ki * candidate
        # Do not integrate into either velocity or acceleration saturation.
        if not ((output > upper and error > 0) or (output < lower and error < 0)):
            self.integral = candidate
        self.previous_error = error
        return max(lower, min(upper, base + self.ki * self.integral))
