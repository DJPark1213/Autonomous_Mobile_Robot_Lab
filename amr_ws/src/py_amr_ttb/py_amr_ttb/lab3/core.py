"""Shared scan/safety utilities and the Lecture 5 comparison controller.

Coordinates: x forward, y left, yaw counterclockwise. Distances are metres.
LectureNavigator follows AMR Lecture 5 pp.45-50: GTG/AO and wall following.
The default continuous controller is selected in navigation.py.
Lecture 3 p.42 supplies the proportional heading/distance control structure.
"""

from dataclasses import dataclass
import math
from typing import ClassVar

import numpy as np


def wrap(angle):
    """Wrap scalar or array angles into [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Config:
    """Validated settings shared by the ROS node and offline simulations."""

    ROBOT_NAMESPACE: ClassVar[str] = '/TTB10'

    goal_x: float = 1.5
    goal_y: float = 0.0
    goal_tolerance: float = 0.06
    planner: str = 'potential'
    max_linear: float = 0.18
    max_angular: float = 0.7
    robot_radius: float = 0.17
    safety_margin: float = 0.08
    lookahead: float = 0.55
    wall_distance: float = 0.40
    wall_gain: float = 4.0
    wall_hysteresis: float = 0.025
    leave_progress: float = 0.03
    field_influence: float = 0.50
    repulsion_gain: float = 0.002
    tangent_gain: float = 4.0
    field_stall_timeout: float = 1.5
    field_goal_fade: float = 0.35
    angular_gain: float = 1.8
    linear_gain: float = 0.6
    sensor_timeout: float = 0.4
    control_period: float = 0.05
    acceleration: float = 0.3
    braking_deceleration: float = 0.5
    progress_timeout: float = 25.0
    max_runtime: float = 120.0
    follow_distance: float = 0.8
    follow_max_range: float = 2.5
    follow_lost_timeout: float = 0.7
    preferred_side: str = 'auto'
    allow_positive_inf: bool = True

    def __post_init__(self):
        """Reject nonsensical settings before a velocity publisher is used."""
        for name, value in vars(self).items():
            if isinstance(value, (float, int)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError(f'{name} must be finite')
                if name not in ('goal_x', 'goal_y') and value <= 0:
                    raise ValueError(f'{name} must be positive')
        if self.preferred_side not in ('auto', 'left', 'right'):
            raise ValueError('preferred_side must be auto, left or right')
        if self.planner not in ('potential', 'lecture'):
            raise ValueError('planner must be potential or lecture')
        if self.max_linear > 0.35 or self.max_angular > 1.5:
            raise ValueError('Lab limits: max_linear <= 0.35, max_angular <= 1.5')
        if self.robot_radius < 0.17 or self.safety_margin < 0.04:
            raise ValueError('Use robot_radius >= 0.17 and safety_margin >= 0.04')
        if self.sensor_timeout < 2 * self.control_period:
            raise ValueError('sensor_timeout must span at least two control ticks')
        if self.lookahead < self.max_linear * self.sensor_timeout + (
                self.max_linear ** 2 / (2 * self.braking_deceleration)):
            raise ValueError('lookahead is shorter than stopping travel')
        if self.follow_distance <= self.radius + 0.15:
            raise ValueError('follow_distance is too small for the footprint')
        if self.follow_max_range <= self.follow_distance:
            raise ValueError('follow_max_range must exceed follow_distance')
        if self.wall_distance - self.wall_hysteresis <= self.radius + .06:
            raise ValueError('wall_distance must leave room beyond the inflated footprint')

    @property
    def radius(self):
        """Return the inflated circular footprint radius."""
        return self.robot_radius + self.safety_margin


@dataclass
class Scan:
    """Processed base-frame hit points and scan-order points for clustering."""

    points: np.ndarray
    ordered: np.ndarray
    healthy: bool
    reason: str
    sectors: dict


def process_scan(ranges, angle_min, angle_increment, range_min, range_max,
                 transform=(0.0, 0.0, 0.0), allow_positive_inf=True, angle_sign=1.):
    """Use message geometry, validate 360-degree coverage and apply planar TF.

    NaN, negative infinity and out-of-range values are unknown. Positive
    infinity may mean no return, but an entirely infinite scan is unhealthy.
    Require >=50% known beams in every 10-degree bin, with at least 3 finite
    hits overall. No interpolation hides an unknown angular sector.
    """
    empty = np.empty((0, 2))
    bad = lambda reason: Scan(empty, empty, False, reason, {})
    meta = (angle_min, angle_increment, range_min, range_max, angle_sign, *transform)
    if not all(math.isfinite(v) for v in meta):
        return bad('non-finite scan metadata/transform')
    if angle_sign not in (-1., 1.):
        return bad('invalid laser orientation')
    if (angle_increment == 0 or abs(angle_increment) > math.radians(5)
            or range_min < 0 or range_max <= range_min or len(ranges) < 72):
        return bad('invalid scan geometry')
    span = abs(angle_increment) * (len(ranges) - 1)
    if not 2 * math.pi - math.radians(12) <= span <= 2 * math.pi + .1:
        return bad('a full 360-degree scan is required')
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(values)) * angle_increment
    finite = np.isfinite(values) & (values >= range_min) & (values <= range_max)
    known = finite | (np.isposinf(values) if allow_positive_inf else False)
    bins = np.floor((wrap(angles) + math.pi) / (2 * math.pi / 36)).astype(int)
    counts = np.bincount(bins, minlength=36)
    valid = np.bincount(bins, weights=known.astype(float), minlength=36)
    if np.any(counts == 0) or np.any(valid < counts * .5):
        return bad('unknown or missing scan sector')
    if np.count_nonzero(finite) < 3:
        return bad('too few finite returns (possible sensor failure)')
    tx, ty, yaw = transform
    # Preserve one entry per original beam; unknown/no-return entries are NaN.
    ordered = np.full((len(values), 2), np.nan)
    ordered[finite, 0] = tx + values[finite] * np.cos(angle_sign*angles[finite] + yaw)
    ordered[finite, 1] = ty + values[finite] * np.sin(angle_sign*angles[finite] + yaw)
    points = ordered[finite]
    bearings = np.arctan2(points[:, 1], points[:, 0])
    distances = np.linalg.norm(points, axis=1)
    sectors = {}
    for i, name in enumerate(('F', 'FL', 'L', 'BL', 'B', 'BR', 'R', 'FR')):
        mask = np.abs(wrap(bearings - i * math.pi / 4)) <= math.pi / 8
        sectors[name] = float(np.min(distances[mask])) if np.any(mask) else math.inf
    return Scan(points, ordered, True, '', sectors)


def free_distances(points, headings, radius, cap=4.0):
    """Distance a circular robot can translate along each heading before a hit."""
    headings = np.atleast_1d(headings)
    if not len(points):
        return np.full(len(headings), cap)
    along = (points[:, 0, None] * np.cos(headings)
             + points[:, 1, None] * np.sin(headings))
    across = (-points[:, 0, None] * np.sin(headings)
              + points[:, 1, None] * np.cos(headings))
    root = np.sqrt(np.maximum(0.0, radius ** 2 - across ** 2))
    collision = (np.abs(across) < radius) & (along + root >= 0)
    distances = np.where(collision, np.maximum(0.0, along - root), cap)
    return np.minimum(cap, distances.min(axis=0))


def guard_command(v, w, scan, config, previous_v=0.0):
    """Stop for unsafe footprint/curved stopping sweep, including old speed.

    The circular footprint allows in-place rotation only when the whole body
    has clearance. The horizon includes sensor latency and braking travel.
    """
    if not scan.healthy:
        return 0.0, 0.0
    points = scan.points
    if len(points) and np.min(np.linalg.norm(points, axis=1)) <= config.radius:
        return 0.0, 0.0
    speed = max(v, previous_v)
    horizon = config.sensor_timeout + speed / config.braking_deceleration
    for t in np.linspace(0, horizon, 15):
        if abs(w) < 1e-6:
            x, y = speed * t, 0.0
        else:
            x = speed * math.sin(w * t) / w
            y = speed * (1 - math.cos(w * t)) / w
        if len(points) and np.any(np.hypot(points[:, 0] - x, points[:, 1] - y)
                                 <= config.radius + .01):
            return 0.0, 0.0
    return float(v), float(w)


class LectureNavigator:
    """Lecture 5 hybrid GTG/AO/FW controller, adapted to a differential drive.

    Closest obstacle: p.50. Wall tangents: p.45. Dot-product side selection:
    p.47. Leave only after progress AND a clear shot: pp.48-49.
    Hysteresis, radial correction, whole-body corridor validation and the
    final velocity guard are explicitly practical additions to the slides.
    """

    def __init__(self, config):
        self.cfg = config
        self.state = 'WAITING'
        self.behavior = 'GO_TO_GOAL'
        # Rotation of the outward obstacle normal: -1 = CW, +1 = CCW.
        self.wall_direction = 0
        self.entry_distance = math.inf
        self.clear_ticks = 0
        self.previous_v = 0.0
        self.done = False
        self.failure = ''
        self.started = None
        self.last_progress = None
        self.best_distance = math.inf
        self.diagnostics = {}

    def pause(self, state='WAITING'):
        """Clear commanded speed after any missing-data stop."""
        self.previous_v = 0.0
        self.state = state
        return 0.0, 0.0

    def select_wall_direction(self, goal, normal, distance):
        """Choose the tangent with maximum goal alignment, per lecture p.47."""
        clockwise = np.array([normal[1], -normal[0]])
        if self.cfg.preferred_side == 'auto':
            self.wall_direction = -1 if np.dot(goal, clockwise) >= 0 else 1
        else:
            # For an obstacle ahead, CW passes on its left and CCW on its right.
            self.wall_direction = -1 if self.cfg.preferred_side == 'left' else 1
        self.entry_distance = distance
        self.clear_ticks = 0

    def wall_behavior(self):
        """Name a boundary-following mode without changing its stored direction."""
        return 'FOLLOW_WALL_CW' if self.wall_direction == -1 else 'FOLLOW_WALL_CCW'

    def step(self, pose, scan, now, dt, mode='goal'):
        """Return bounded (linear, angular) velocity; completion is latched."""
        if self.done or self.failure:
            return self.pause('DONE' if self.done else self.failure)
        if not scan.healthy:
            return self.pause('SCAN_INVALID')
        if not all(math.isfinite(v) for v in (*pose, now, dt)) or dt <= 0:
            return self.pause('INVALID_INPUT')
        if dt > self.cfg.sensor_timeout:
            return self.pause('TIMING_GAP')
        x, y, yaw = pose
        distance = math.hypot(self.cfg.goal_x - x, self.cfg.goal_y - y)
        if mode == 'goal' and distance <= self.cfg.goal_tolerance:
            self.done = True
            return self.pause('DONE')
        if self.started is None:
            self.started = self.last_progress = now
        if mode == 'goal':
            if distance < self.best_distance - .03:
                self.best_distance = distance
                self.last_progress = now
            if now - self.last_progress > self.cfg.progress_timeout:
                self.failure = 'NO_PROGRESS'
            if now - self.started > self.cfg.max_runtime:
                self.failure = 'TIME_LIMIT'
            if self.failure:
                return self.pause(self.failure)
        if np.min(np.linalg.norm(scan.points, axis=1)) <= self.cfg.radius:
            return self.pause('TOO_CLOSE')
        # Express BOTH vectors in the same base frame before dot products.
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = self.cfg.goal_x-x, self.cfg.goal_y-y
        goal = np.array([c*dx + s*dy, -s*dx + c*dy]) if mode == 'goal' else np.array([1., 0.])
        goal_heading = math.atan2(goal[1], goal[0])
        lengths = np.linalg.norm(scan.points, axis=1)
        nearest = scan.points[np.argmin(lengths)]
        obstacle_distance = float(lengths.min())
        normal = -nearest / obstacle_distance  # u_AO direction, away from obstacle.
        clockwise = np.array([normal[1], -normal[0]])
        clear_shot = float(np.dot(normal, goal)) > 0
        # A finite robot additionally needs a clear corridor all the way to goal.
        check_length = distance if mode == 'goal' else self.cfg.lookahead
        direct = free_distances(scan.points, [goal_heading], self.cfg.radius + .03,
                                cap=max(4., check_length+.1))[0]
        corridor_clear = direct >= check_length
        self.diagnostics = {
            'obstacle_distance_m': obstacle_distance,
            'wall_entry_goal_distance_m': self.entry_distance,
            'goal_normal_dot': float(np.dot(normal, goal)),
            'goal_corridor_clear': corridor_clear,
        }
        if self.behavior == 'GO_TO_GOAL':
            approaching = not clear_shot or not corridor_clear
            if (obstacle_distance <= self.cfg.wall_distance + self.cfg.wall_hysteresis
                    and approaching):
                self.select_wall_direction(goal, normal, distance)
                self.behavior = self.wall_behavior()

        # Lecture p.49: enter AO inside the wall-distance boundary; hysteresis
        # replaces exact distance equalities that would chatter with real scans.
        if obstacle_distance < self.cfg.wall_distance - self.cfg.wall_hysteresis:
            if not self.wall_direction:
                self.select_wall_direction(goal, normal, distance)
            self.behavior = 'AVOID_OBSTACLE'
        elif self.behavior == 'AVOID_OBSTACLE':
            if obstacle_distance >= self.cfg.wall_distance:
                self.behavior = self.wall_behavior()
                self.entry_distance = distance
                self.clear_ticks = 0

        if self.behavior.startswith('FOLLOW_WALL'):
            progress = distance < self.entry_distance - self.cfg.leave_progress
            may_leave = (progress and clear_shot and corridor_clear) if mode == 'goal' else corridor_clear
            self.clear_ticks = self.clear_ticks + 1 if may_leave else 0
            if self.clear_ticks >= 3:
                self.behavior = 'GO_TO_GOAL'
                self.wall_direction = 0
                self.clear_ticks = 0

        if self.behavior == 'GO_TO_GOAL':
            direction = goal
        elif self.behavior == 'AVOID_OBSTACLE':
            # An enclosed robot must not keep spinning toward an impossible escape.
            escape_heading = math.atan2(normal[1], normal[0])
            escape = free_distances(scan.points, [escape_heading], self.cfg.radius+.03)[0]
            if escape < .1:
                return self.pause('BLOCKED')
            direction = normal
        else:
            tangent = clockwise if self.wall_direction == -1 else -clockwise
            radial = np.clip(self.cfg.wall_gain * (self.cfg.wall_distance-obstacle_distance), -.7, .7)
            direction = tangent + radial*normal

        # Lecture 3 p.42, with saturation and turn-first gating for this robot.
        heading = math.atan2(direction[1], direction[0])
        w = float(np.clip(self.cfg.angular_gain * heading,
                          -self.cfg.max_angular, self.cfg.max_angular))
        speed = self.cfg.max_linear
        if mode == 'goal':
            speed = min(speed, self.cfg.linear_gain * distance)
        if self.behavior == 'AVOID_OBSTACLE':
            speed *= .6
        # Turn first for a large heading change; slow down during arcs.
        v = speed * max(0.0, math.cos(heading)) ** 2 if abs(heading) < .55 else 0.0
        v = min(v, self.previous_v + self.cfg.acceleration * dt)
        v, w = guard_command(v, w, scan, self.cfg, self.previous_v)
        self.previous_v = v
        self.state = ('SAFETY_STOP' if v == 0 and w == 0 else
                      'WANDER' if mode == 'wander' and self.behavior == 'GO_TO_GOAL'
                      else self.behavior)
        return v, w
