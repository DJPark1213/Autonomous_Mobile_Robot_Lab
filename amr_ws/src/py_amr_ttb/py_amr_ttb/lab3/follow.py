"""Optional lidar-cluster following; this does not identify a human person."""

import math

import numpy as np

from .core import guard_command, wrap


def clusters(scan, max_range):
    """Group adjacent scan hits, rejecting walls and tiny isolated returns."""
    groups = []
    current = []
    for point in scan.ordered:
        usable = np.all(np.isfinite(point)) and .25 <= np.linalg.norm(point) <= max_range
        if not usable or (current and np.linalg.norm(point - current[-1]) > .16):
            if current:
                groups.append(np.asarray(current))
            current = []
        if usable:
            current.append(point)
    if current:
        groups.append(np.asarray(current))
    # Merge a cluster split at the scan's +/-pi seam, only if end beams are valid.
    if (len(groups) > 1 and np.all(np.isfinite(scan.ordered[[0, -1]]))
            and np.linalg.norm(groups[0][0] - groups[-1][-1]) <= .16):
        groups[0] = np.concatenate((groups[-1], groups[0]))
        groups.pop()
    candidates = []
    for group in groups:
        if len(group) < 3:
            continue
        width = float(np.linalg.norm(np.ptp(group, axis=0)))
        if .04 <= width <= .75:
            candidates.append(np.median(group, axis=0))
    return candidates


def to_world(point, pose):
    """Transform a base-frame target into the fixed odometry-relative frame."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x + c * point[0] - s * point[1],
                     y + s * point[0] + c * point[1]])


class Follower:
    """Acquire a central cluster for five scans; stop immediately when lost.

    Association is spatial in the fixed frame so robot motion is compensated.
    After a prolonged loss, explicit reset is required to acquire a new target.
    """

    def __init__(self, config):
        self.cfg = config
        self.target = None
        self.last_seen = None
        self.acquired = False
        self.confirmations = 0
        self.visible = False
        self.lost_latched = False
        self.previous_v = 0.0
        self.state = 'ACQUIRE_TARGET'

    def observe(self, scan, pose, now):
        """Update once per scan, never once per control tick."""
        self.visible = False
        if self.lost_latched or not scan.healthy:
            return
        if self.last_seen is not None and now - self.last_seen > self.cfg.follow_lost_timeout:
            if self.acquired:
                self.lost_latched = True
                self.state = 'TARGET_LOST_RESET_REQUIRED'
                return
            self.target = None
            self.confirmations = 0
        candidates = clusters(scan, self.cfg.follow_max_range)
        if self.target is None:
            candidates = [p for p in candidates if p[0] > .4
                          and abs(math.atan2(p[1], p[0])) < math.radians(20)]
            if not candidates:
                return
            point = min(candidates, key=lambda p: abs(math.atan2(p[1], p[0])) + .1*np.linalg.norm(p))
            selected = to_world(point, pose)
        else:
            world = [to_world(p, pose) for p in candidates]
            if not world:
                return
            selected = min(world, key=lambda p: np.linalg.norm(p - self.target))
            elapsed = max(.0, now - self.last_seen)
            # Tighten acquisition to avoid confirming different objects.
            gate = min(.45, .12 + 1.2 * elapsed)
            if np.linalg.norm(selected - self.target) > gate:
                return
        self.target = selected
        self.last_seen = now
        self.visible = True
        self.confirmations += 1
        self.acquired = self.confirmations >= 5

    def step(self, pose, scan, now, dt):
        """Follow with a distance deadband and the common collision guard."""
        if self.last_seen is not None and now - self.last_seen > self.cfg.follow_lost_timeout:
            self.lost_latched = self.acquired or self.lost_latched
        if self.lost_latched:
            self.state = 'TARGET_LOST_RESET_REQUIRED'
        elif not self.acquired:
            self.state = 'ACQUIRE_TARGET'
        elif not self.visible or now - self.last_seen > self.cfg.sensor_timeout:
            self.state = 'TARGET_NOT_VISIBLE'
        elif not scan.healthy or dt <= 0 or dt > self.cfg.sensor_timeout:
            self.state = 'SENSOR_STOP'
        else:
            dx, dy = self.target - np.asarray(pose[:2])
            distance = math.hypot(dx, dy)
            heading = float(wrap(math.atan2(dy, dx) - pose[2]))
            error = distance - self.cfg.follow_distance
            v = min(self.cfg.max_linear, max(0.0, .5 * error)) if error > .08 else 0.0
            v = v * max(0.0, math.cos(heading)) if abs(heading) < .55 else 0.0
            v = min(v, self.previous_v + self.cfg.acceleration * dt)
            w = float(np.clip(self.cfg.angular_gain * heading,
                              -self.cfg.max_angular, self.cfg.max_angular))
            v, w = guard_command(v, w, scan, self.cfg, self.previous_v)
            self.previous_v = v
            self.state = 'FOLLOW' if v or w else 'HOLD_DISTANCE_OR_OBSTACLE'
            return v, w
        self.previous_v = 0.0
        return 0.0, 0.0
