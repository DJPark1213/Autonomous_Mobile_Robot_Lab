"""Start the Lab 3 1.5-m obstacle-avoidance task."""

from py_amr_ttb.lab3.launch_support import launch_description


def generate_launch_description():
    """Return the goal-mode launch description."""
    return launch_description('goal')
