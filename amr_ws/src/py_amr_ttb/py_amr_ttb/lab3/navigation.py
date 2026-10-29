"""Select a reproducible lecture baseline or the continuous potential controller."""

from .core import LectureNavigator


def make_navigator(config):
    """Construct the configured controller; the ROS node and simulator share it."""
    if config.planner == 'lecture':
        return LectureNavigator(config)
    from .potential import PotentialNavigator
    return PotentialNavigator(config)
