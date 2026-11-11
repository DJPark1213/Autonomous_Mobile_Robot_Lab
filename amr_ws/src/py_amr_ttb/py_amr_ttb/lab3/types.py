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
    """The same destination needs different steering when approaching a
    wall, backing out or braking at the final goal.

    The planner supplies point in the map frame; the controller adapter converts it
    to odometry. mode selects the behavior, reverse permits backward recovery, and
    wall_side is +1 left or -1 right. score ranks candidate targets.
    """

    point: tuple
    mode: str = 'FIRST_RUN'
    score: float = 0.0
    reverse: bool = False
    wall_side: int = 0


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
    """A map correction made after target selection could send the
    robot toward the wrong odometry location if target and transform were read
    separately.

    Keep the map-frame target with transform, which locates starting odometry in the
    map; control uses its inverse. created uses monotonic time for expiry. status
    and diagnostics explain the decision.
    """

    target: Target | None
    transform: tuple
    created: float
    map_revision: int = 0
    status: str = 'RUNNING'
    diagnostics: dict = field(default_factory=dict)
