"""A scan placed at the wrong robot pose can turn a stationary wall
into a shifted obstacle in the local scene.

StateTracker matches scan times to starting-frame odometry. PoseEstimator separately
applies accepted scan corrections for map-backed replay.
"""

from collections import deque
import math
import numpy as np

from py_amr_ttb.odom_utils import normalize_angle, relative_pose, odometry_is_fresh
from .types import RobotState
from .geometry import between, compose


class StateTracker:
    """A run needs a fixed origin and pose history even when incoming
    odometry uses a different origin or the laser arrives late.

    Keep starting-frame measurements for interpolation. Reject frame changes,
    backward timestamps and implausible jumps rather than silently relocating an
    existing route and scan history.
    """

    def __init__(self, config):
        self.cfg = config
        self.origin = None
        self.frame = None
        self.samples = deque(maxlen=200)

    @property
    def latest(self):
        return self.samples[-1] if self.samples else None

    def update(self, pose, v, w, stamp, received, frame='odom', covariance=()):
        """An odometry reset or frame jump would otherwise move the
        robot relative to the goal, recorded route and previous scans.

        Validate continuity and retain a RobotState in the fixed starting frame.
        pose uses the named odometry frame; measured v and w keep their signs.
        Invalid input raises ValueError so the adapter can require reset.
        """
        if not frame or not all(math.isfinite(z) for z in (*pose, v, w, stamp, received)):
            raise ValueError('Invalid odometry or measured motion')
        if self.origin is None:
            # Preserve this anchor so the goal and past samples keep their meaning.
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
        """The newest odometry pose may be later than a laser
        measurement, especially while the robot is moving.

        Return the pose at the scan's stamp by interpolation with the short angular
        difference. Outside the history, extrapolate from the nearest measured
        speeds only within pose_sync_tolerance; otherwise raise ValueError.
        """
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
        """The last pose can still look valid after odometry messages
        stop arriving.

        Return whether the newest state exists and its monotonic age is within
        sensor_timeout.
        """
        return self.latest is not None and odometry_is_fresh(
            now, self.latest.received, self.cfg.sensor_timeout)


class PoseEstimator:
    """Accumulated odometry error can place repeated views of the same
    wall at different map locations.

    Predict with odometry increments and correct with accepted scan matches.
    Covariance weights corrections and rejects implausible jumps. Because the map
    depends on past poses, this is a local uncertainty estimate, not a full SLAM
    posterior or certified error bound.
    """

    def __init__(self, config):
        self.cfg = config
        self.pose = (0., 0., 0.)
        self.previous = None
        self.reported = None
        self.covariance = np.diag([.01**2, .01**2, .02**2])

    def predict(self, odom_pose, reported=()):
        """Replacing a corrected map pose with absolute odometry would
        discard earlier scan corrections.

        Apply only the movement since previous odom_pose and grow uncertainty with
        that motion. Add valid increases in reported covariance rather than its full
        cumulative value. Return the predicted map pose.
        """
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
                    # Adding cumulative covariance each scan would count the
                    # same uncertainty repeatedly; add only its increases.
                    noise += np.diag(np.maximum(0., values-self.reported))
                self.reported = values
        self.pose = compose(self.pose, delta)
        self.covariance = jacobian@self.covariance@jacobian.T+noise
        self.previous = tuple(odom_pose)
        return self.pose

    def correct(self, measured):
        """Repeated-looking walls can produce a plausible match score
        at an implausible robot pose.

        Gate the map-frame (x, y, yaw) measurement against predicted uncertainty.
        Return False without correction if rejected; otherwise apply the Kalman
        update and return True.
        """
        residual = np.asarray(measured)-self.pose
        residual[2] = normalize_angle(residual[2])
        variance = np.diag([self.cfg.alignment_position_noise**2]*2+
                           [self.cfg.alignment_heading_noise**2])
        innovation = self.covariance+variance
        # A strong wall score can still imply an implausible pose jump. Gate the
        # correction against current uncertainty before moving the map estimate.
        if float(residual@np.linalg.solve(innovation, residual)) > self.cfg.innovation_limit:
            return False
        gain = np.linalg.solve(innovation.T, self.covariance.T).T
        corrected = np.asarray(self.pose)+gain@residual
        corrected[2] = normalize_angle(corrected[2])
        self.pose = tuple(corrected)
        identity = np.eye(3)-gain
        # Roundoff can damage covariance symmetry and positivity; this update
        # form is more numerically robust than simply subtracting the gain term.
        self.covariance = identity@self.covariance@identity.T+gain@variance@gain.T
        return True

    def initialize(self, odom_pose, map_pose):
        """Replay starts with new odometry coordinates but must locate
        the robot in the previously learned map.

        Pair odom_pose with the accepted map_pose and initialize correction
        uncertainty.
        """
        self.previous, self.pose = tuple(odom_pose), tuple(map_pose)
        self.covariance = np.diag([self.cfg.alignment_position_noise**2]*2+
                                  [self.cfg.alignment_heading_noise**2])
