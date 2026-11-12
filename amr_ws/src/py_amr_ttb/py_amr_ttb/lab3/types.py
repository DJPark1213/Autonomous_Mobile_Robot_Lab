"""A value can be misused if its frame, timestamp or validity is lost
while passing between sensing, planning and control.

These records carry that context through the ROS adapter, planning worker and
simulator.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Scan:
    """A finite stored distance can represent an invalid reading, so
    distances alone cannot tell collision checks what the laser actually observed.

    angles, ranges, known and hits are matching beam arrays. known identifies usable
    coverage and hits identifies obstacle returns inside the usable limit. Geometry
    uses the robot frame located by pose in starting odometry; origin is the laser
    position in that frame.

    pose initially refers to acquisition and changes if the geometry is re-
    expressed. stamp remains acquisition time, received is the monotonic freshness
    reference, and sequence helps avoid repeated map insertion.
    """

    angles: np.ndarray
    ranges: np.ndarray
    known: np.ndarray
    hits: np.ndarray
    origin: np.ndarray
    stamp: float
    received: float
    pose: tuple
    sequence: int
    history: tuple = ()
    coverage: object = None
    coverage_time: float = 0.
    coverage_timeout: float = .4

    @property
    def endpoints(self):
        """Map insertion needs beam endpoints for both obstacle hits
        and usable no-return beams.

        Return N [x, y] rows in the scan frame, including placeholders. Callers must
        use known and hits to distinguish which endpoints carry evidence.
        """
        directions = np.column_stack((np.cos(self.angles), np.sin(self.angles)))
        return self.origin + self.ranges[:, None] * directions

    @property
    def points(self):
        """Collision checks must not mistake clipped no-return
        distances or unknown-beam placeholders for actual obstacles.

        Return only endpoints selected by hits. An empty result alone does not
        establish free space.
        """
        return self.endpoints[self.hits]


@dataclass
class RobotState:
    """Stopping prediction needs the robot's measured motion; its
    requested velocity may not yet have been reached.

    pose is (x, y, yaw) in starting odometry; v and w are signed measured m/s and
    rad/s. stamp uses sensor time, received uses monotonic time for freshness, and
    covariance carries reported pose uncertainty.
    """

    pose: tuple
    v: float
    w: float
    stamp: float
    received: float
    covariance: tuple = ()


@dataclass
class Target:
    """A persistent objective in the plan's coordinate frame."""

    point: tuple
    mode: str = 'GOAL'

@dataclass
class MotionResult:
    """Planning needs to know when control could not follow a target,
    and logs need the reason for a slowdown or stop.

    v and w are commands in m/s and rad/s; reason explains them and blocked requests
    reconsideration. trajectory stores predicted poses when evaluated. Defaults
    request zero velocity.
    """

    v: float = 0.0
    w: float = 0.0
    reason: str = 'WAITING'
    blocked: bool = False
    trajectory: list = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)


@dataclass
class Plan:
    """Keep the selected objective and executable command in one snapshot.

    transform locates starting odometry in the target's frame for telemetry.
    command is (linear m/s, angular rad/s); None authorizes no movement.
    created uses the input's monotonic time so computation cannot renew old evidence.
    """

    target: Target | None
    transform: tuple
    created: float
    status: str = 'RUNNING'
    diagnostics: dict = field(default_factory=dict)
    command: tuple | None = None
