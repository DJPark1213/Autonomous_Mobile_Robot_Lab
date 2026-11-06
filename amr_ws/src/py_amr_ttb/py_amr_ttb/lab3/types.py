"""Timestamped values shared by the robot adapter, worker and simulator."""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class Scan:
    """Laser geometry in the robot base frame identified by pose.

    The four beam arrays share the same bearing order. known distinguishes
    usable observations from missing data; hits selects actual obstacles.
    ranges holds finite lengths even for unknown beams, so a length alone
    must never be treated as evidence of free space. Re-expressing the geometry
    changes pose, but the acquisition stamp remains unchanged.
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

    @property
    def endpoints(self):
        """Return Nx2 beam endpoints, including placeholders for unknown rays."""
        directions = np.column_stack((np.cos(self.angles), np.sin(self.angles)))
        return self.origin + self.ranges[:, None] * directions

    @property
    def points(self):
        """Return only observed obstacle-hit points in the scan's base frame."""
        return self.endpoints[self.hits]


@dataclass
class RobotState:
    """Continuous starting-odometry pose, signed speeds and reported covariance."""

    pose: tuple
    v: float
    w: float
    stamp: float
    received: float
    covariance: tuple = ()


@dataclass
class Target:
    """A point in the map frame; wall side is +1 left, -1 right."""

    point: tuple
    mode: str = 'FIRST_RUN'
    score: float = 0.0
    reverse: bool = False
    wall_side: int = 0


@dataclass
class MotionResult:
    v: float = 0.0
    w: float = 0.0
    reason: str = 'WAITING'
    blocked: bool = False
    trajectory: list = field(default_factory=list)


@dataclass
class Plan:
    """Worker output: a target and its exact map-to-odometry transform."""

    target: Target | None
    transform: tuple
    created: float
    map_revision: int = 0
    status: str = 'RUNNING'
    diagnostics: dict = field(default_factory=dict)
