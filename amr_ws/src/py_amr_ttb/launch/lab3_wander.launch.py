"""Start local obstacle-avoidance practice without a fixed goal."""

from py_amr_ttb.lab3.launch_support import launch_description


def generate_launch_description():
    """Return the wander-mode launch description."""
    return launch_description('wander')
