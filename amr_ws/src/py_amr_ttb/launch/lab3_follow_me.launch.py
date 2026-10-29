"""Start the optional lidar-cluster following task."""

from py_amr_ttb.lab3.launch_support import launch_description


def generate_launch_description():
    """Return the follow-mode launch description."""
    return launch_description('follow')
