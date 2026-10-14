"""Check starting-frame geometry independently of ROS transport."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


spec = importlib.util.spec_from_file_location(
    'odom_utils',
    Path(__file__).parents[1] / 'py_amr_ttb' / 'odom_utils.py',
)
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)


class StartingFrameTests(unittest.TestCase):
    """Verify rotated origins, heading wrap, and reset-equivalent inputs."""

    def test_rotated_translated_start(self):
        # Facing world +y: world +y is forward and world -x is left.
        pose = utils.relative_pose(8.5, 21.5, math.pi, 10, 20, math.pi / 2)
        for actual, expected in zip(pose, (1.5, 1.5, math.pi / 2)):
            self.assertAlmostEqual(actual, expected)

    def test_start_is_zero(self):
        self.assertEqual(utils.relative_pose(4, 7, 1, 4, 7, 1), (0, 0, 0))

    def test_heading_wrap(self):
        pose = utils.relative_pose(0, 0, -math.pi + .1, 0, 0, math.pi - .1)
        self.assertAlmostEqual(pose[2], .2)

    def test_quaternion_heading(self):
        q = SimpleNamespace(x=0, y=0, z=math.sin(math.pi / 4),
                            w=math.cos(math.pi / 4))
        self.assertAlmostEqual(utils.quaternion_to_yaw(q), math.pi / 2)


if __name__ == '__main__':
    unittest.main()
