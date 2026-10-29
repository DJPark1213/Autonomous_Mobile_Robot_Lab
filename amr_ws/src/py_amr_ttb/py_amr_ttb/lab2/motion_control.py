"""Independent alignment recovery, circle speed regulation, and goal PID."""

import math

from py_amr_ttb.lab2.pid import PID


class AlignmentRecovery:
    """Hysteresis prevents chatter between forward tracking and alignment."""

    def __init__(self, enter, exit):
        """Configure distinct entry and exit heading thresholds."""
        if not 0 < exit < enter <= math.pi:
            raise ValueError('require 0 < align_exit < align_enter <= pi')
        self.enter, self.exit = enter, exit
        self.reset()

    def reset(self):
        """Mark motion stopped before the next tracking decision."""
        self.state = 'STOP'

    def update(self, heading_error):
        """Select FOLLOW or ALIGN using the current state and heading."""
        if self.state == 'ALIGN':
            if abs(heading_error) <= self.exit:
                self.state = 'FOLLOW'
        elif abs(heading_error) >= self.enter:
            self.state = 'ALIGN'
        else:
            self.state = 'FOLLOW'
        return self.state


class CircleSpeedController:
    """Regulate speed while respecting curvature and command ramp limits."""

    def __init__(self, kp, ki, kd, acceleration):
        """Create a speed PID and acceleration limit in m/s squared."""
        self.pid = PID(kp, ki, kd)
        self.acceleration = acceleration
        self.reset()

    def reset(self):
        """Clear speed history and restart the command ramp from zero."""
        self.pid.reset()
        self.command = 0.0

    def update(self, desired, measured, curvature, dt, max_linear, max_angular):
        """Return feasible forward speed with feedforward and PID feedback."""
        limit = max_linear
        if abs(curvature) > 1e-12:
            limit = min(limit, max_angular / abs(curvature))
        target = min(desired, limit)
        lower = max(0.0, self.command - self.acceleration * dt)
        upper = min(limit, self.command + self.acceleration * dt)
        # A newly tighter curvature limit takes precedence over the ramp.
        lower = min(lower, upper)
        self.command = self.pid.update(target - measured, dt, lower, upper, target)
        return self.command


class GoalController:
    """Steer toward the goal while scaling forward speed by heading error."""

    def __init__(self, config, linear_ki, linear_kd, angular_ki, angular_kd):
        """Create independent distance and heading PID histories."""
        self.config = config
        self.distance_pid = PID(config.NAV_LINEAR_GAIN, linear_ki, linear_kd)
        self.heading_pid = PID(config.NAV_ANGULAR_GAIN, angular_ki, angular_kd,
                               angular=True)

    def reset(self):
        """Clear control and alignment state on a stop."""
        self.distance_pid.reset()
        self.heading_pid.reset()

    def update(self, x, y, yaw, tx, ty, dt):
        """Return linear/angular command and arrival flag for one target."""
        distance = math.hypot(tx - x, ty - y)
        if distance <= self.config.NAV_POSITION_TOLERANCE:
            self.reset()
            return 0.0, 0.0, True
        error = math.atan2(ty - y, tx - x) - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        angular = self.heading_pid.update(error, dt, -self.config.NAV_MAX_ANGULAR_SPEED,
                                          self.config.NAV_MAX_ANGULAR_SPEED)
        # Drive along an arc for targets ahead; turn in place only when the
        # target is at least 90 degrees away, avoiding motion away from it.
        speed_limit = self.config.NAV_MAX_LINEAR_SPEED * max(0.0, math.cos(error))
        if speed_limit == 0.0:
            self.distance_pid.reset()
            return 0.0, angular, False
        linear = self.distance_pid.update(distance, dt, 0.0, speed_limit)
        return linear, angular, False
