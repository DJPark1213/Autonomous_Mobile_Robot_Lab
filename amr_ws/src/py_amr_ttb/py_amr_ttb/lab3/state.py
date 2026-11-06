"""Continuous odometry, signed motion, and interpolation at laser acquisition time."""

from collections import deque
import math
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle, relative_pose, odometry_is_fresh
from .types import RobotState
from .geometry import between, compose


class StateTracker:
    def __init__(self, config):
        self.cfg = config
        self.origin = None
        self.frame = None
        self.samples = deque(maxlen=200)

    @property
    def latest(self):
        return self.samples[-1] if self.samples else None

    def update(self, pose, v, w, stamp, received, frame='odom', covariance=()):
        if not frame or not all(math.isfinite(z) for z in (*pose, v, w, stamp, received)):
            raise ValueError('Invalid odometry or measured motion')
        if self.origin is None:
            self.origin, self.frame = tuple(pose), frame
        if self.frame != frame:
            raise ValueError('Odometry frame changed; reset required')
        local = relative_pose(*pose, *self.origin)
        old = self.latest
        if old:
            dt = stamp-old.stamp
            if dt <= 0:
                raise ValueError('Non-increasing odometry timestamp')
            if (math.dist(local[:2], old.pose[:2]) > .12+0.6*dt
                    or abs(normalize_angle(local[2]-old.pose[2])) > .25+2.5*dt):
                raise ValueError('Odometry jump; reset required')
        state = RobotState(local, float(v), float(w), stamp, received, tuple(covariance))
        self.samples.append(state)
        return state

    def pose_at(self, stamp):
        """Interpolate bracketed poses; allow only bounded velocity extrapolation."""
        if not self.samples:
            raise ValueError('Waiting for odometry')
        items = list(self.samples)
        for a, b in zip(items, items[1:]):
            if a.stamp <= stamp <= b.stamp:
                f = (stamp-a.stamp)/(b.stamp-a.stamp)
                return (a.pose[0]+f*(b.pose[0]-a.pose[0]),
                        a.pose[1]+f*(b.pose[1]-a.pose[1]),
                        normalize_angle(a.pose[2]+f*normalize_angle(b.pose[2]-a.pose[2])))
        nearest = min(items, key=lambda s: abs(s.stamp-stamp))
        dt = stamp-nearest.stamp
        if abs(dt) > self.cfg.pose_sync_tolerance:
            raise ValueError('Scan has no synchronized odometry')
        x, y, yaw = nearest.pose
        return (x+nearest.v*math.cos(yaw+nearest.w*dt/2)*dt,
                y+nearest.v*math.sin(yaw+nearest.w*dt/2)*dt,
                normalize_angle(yaw+nearest.w*dt))

    def fresh(self, now):
        return self.latest is not None and odometry_is_fresh(
            now, self.latest.received, self.cfg.sensor_timeout)


class PoseEstimator:
    """Small planar EKF: odometry predicts; an accepted scan match corrects.

    The map is correlated with past poses, so this covariance is a practical local
    uncertainty estimate, not a full SLAM posterior or a certified error bound.
    """

    def __init__(self, config):
        self.cfg = config
        self.pose = (0., 0., 0.)
        self.previous = None
        self.reported = None
        self.covariance = np.diag([.01**2, .01**2, .02**2])

    def predict(self, odom_pose, reported=()):
        if self.previous is None:
            self.previous = tuple(odom_pose)
            self.pose = tuple(odom_pose)
        delta = between(self.previous, odom_pose)
        x, y, _ = delta
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        jacobian = np.array([[1., 0., -s*x-c*y], [0., 1., c*x-s*y], [0., 0., 1.]])
        distance = math.hypot(x, y)
        translation = self.cfg.odom_translation_noise**2*max(distance, .001)
        rotation = self.cfg.odom_rotation_noise**2*max(abs(delta[2]), distance, .001)
        noise = np.diag([translation, translation, rotation])
        if len(reported) == 36:
            values = np.asarray(reported, dtype=float)[[0, 7, 35]]
            if np.isfinite(values).all() and np.all(values >= 0):
                if self.reported is not None:
                    # Use increases; adding the full cumulative covariance each
                    # scan would count the same uncertainty repeatedly.
                    noise += np.diag(np.maximum(0., values-self.reported))
                self.reported = values
        self.pose = compose(self.pose, delta)
        self.covariance = jacobian@self.covariance@jacobian.T+noise
        self.previous = tuple(odom_pose)
        return self.pose

    def correct(self, measured):
        residual = np.asarray(measured)-self.pose
        residual[2] = normalize_angle(residual[2])
        variance = np.diag([self.cfg.alignment_position_noise**2]*2+
                           [self.cfg.alignment_heading_noise**2])
        innovation = self.covariance+variance
        if float(residual@np.linalg.solve(innovation, residual)) > self.cfg.innovation_limit:
            return False
        gain = np.linalg.solve(innovation.T, self.covariance.T).T
        corrected = np.asarray(self.pose)+gain@residual
        corrected[2] = normalize_angle(corrected[2])
        self.pose = tuple(corrected)
        identity = np.eye(3)-gain
        self.covariance = identity@self.covariance@identity.T+gain@variance@gain.T
        return True

    def initialize(self, odom_pose, map_pose):
        self.previous, self.pose = tuple(odom_pose), tuple(map_pose)
        self.covariance = np.diag([self.cfg.alignment_position_noise**2]*2+
                                  [self.cfg.alignment_heading_noise**2])
