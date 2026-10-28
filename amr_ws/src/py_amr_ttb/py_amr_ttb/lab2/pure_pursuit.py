    radius, direction = path.radius, path.direction
    rx, ry = x, y - direction * radius
    rho = math.hypot(rx, ry)
    phase_error = target_phase - robot_phase  # deliberately NOT wrapped
    v_ref = max(0.0, min(limit, target_speed + .5 * radius * phase_error))
    if v_ref == 0:
        return 0.0, 0.0
    radial_gain = 1.0 / lookahead
    if rho < .05 * radius:
        # The circle's center has no unique tangent. Escape outward in a fixed
        # world direction; unlike chasing a fast target, this bearing is fixed.
        desired_yaw = -direction * math.pi / 2
        coefficient = 0.0
    else:
        ex, ey = rx / rho, ry / rho
        a = radial_gain * (rho - radius)
        qx, qy = -direction * ey - a * ex, direction * ex - a * ey
        desired_yaw = math.atan2(qy, qx)
        c, s = math.cos(yaw), math.sin(yaw)
        # Analytic derivative of field heading along the actual robot motion:
        # psi_dot = polar_dot + d*k_r*rho_dot/(1+(k_r*e_r)^2).
        coefficient = ((ex * s - ey * c) / rho
                       + direction * radial_gain * (ex * c + ey * s) / (1 + a*a))
    heading_error = normalize_angle(desired_yaw - yaw)
    feedback = max(-max_angular, min(max_angular, 1.2 * heading_error))
    linear = v_ref * max(0.0, math.cos(heading_error))
    if abs(coefficient) > 1e-9:
        linear = min(linear, max(0.0, max_angular - abs(feedback)) / abs(coefficient))
    angular = max(-max_angular, min(max_angular, coefficient * linear + feedback))
    return linear, angular


class TimedCircleTarget:
    """Timed target; lookahead sets only its initial chord distance.

    No robot pose/progress is an input. Speed changes preserve target phase.
    """

    def __init__(self, radius, speed=.12, lookahead=.30, direction=1,
                 laps=1, duration=0.0):
        self.path = CirclePath(radius, direction)
        if not math.isfinite(speed) or speed < 0:
            raise ValueError('target_speed must be finite and nonnegative')
        if not math.isfinite(lookahead) or not 0 < lookahead < 2 * radius:
            raise ValueError('lookahead must be positive and less than 2 * radius')
        if isinstance(laps, bool) or not isinstance(laps, int) or laps < 1:
            raise ValueError('laps must be a positive integer')
        if not math.isfinite(duration) or duration < 0:
            raise ValueError('duration must be finite and nonnegative')
        self.lookahead = lookahead
        self.speed = speed
        self.initial_phase = 2 * math.asin(lookahead / (2 * radius))
        self.phase = self.initial_phase
        self.laps = laps
        self.duration = duration
        self.elapsed = 0.0
        self.last_time = None
        self.done = False

    def start(self, now):
        """Fix the clock once after the first valid robot origin is captured."""
        if not math.isfinite(now):
            raise ValueError('time must be finite')
        if self.last_time is None:
            self.last_time = now

    def advance(self, now, speed=None):
        """Integrate old speed to now; a new speed begins on this tick."""
        if not math.isfinite(now):
            raise ValueError('time must be finite')
        if speed is not None and (not math.isfinite(speed) or speed < 0):
            raise ValueError('target_speed must be finite and nonnegative')
        self.start(now)
        dt = now - self.last_time
        if dt < 0:
            raise ValueError('target clock must be monotonic')
        self.last_time = now
        if not self.done:
            if self.duration > 0:
                dt = min(dt, max(0.0, self.duration - self.elapsed))
                self.phase += self.speed * dt / self.path.radius
                self.elapsed += dt
                self.done = self.elapsed >= self.duration - 1e-10
            else:
                remaining = max(0.0, math.tau * self.laps
                                - (self.phase - self.initial_phase))
                advance = self.speed * dt / self.path.radius
                if advance >= remaining - 1e-12 and self.speed > 0:
                    self.elapsed += remaining * self.path.radius / self.speed
                    self.phase = self.initial_phase + math.tau * self.laps
                    self.done = True
                else:
                    self.phase += advance
                    self.elapsed += dt
        if speed is not None:
            self.speed = speed
        return self.path.point(self.phase)

    @property
    def completed_laps(self):
        """Return target revolutions excluding the one-time initial lead."""
        return (self.phase - self.initial_phase) / math.tau
