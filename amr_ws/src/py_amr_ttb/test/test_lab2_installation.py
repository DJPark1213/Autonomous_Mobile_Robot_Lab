"""Check the actual setup manifest against package-local launch files."""

import ast
import os
from pathlib import Path
import runpy
import sys
from types import ModuleType
import unittest
from unittest.mock import patch


class InstallationTests(unittest.TestCase):
    """Catch launch files outside the install glob and stale entry points."""

    def test_setup_installs_all_task_launches_and_navigation_entry_points(self):
        package = Path(__file__).resolve().parents[1]
        captured = {}
        setuptools = ModuleType('setuptools')
        setuptools.find_packages = lambda **kwargs: []
        setuptools.setup = lambda **kwargs: captured.update(kwargs)
        previous = Path.cwd()
        try:
            os.chdir(package)
            with patch.dict(sys.modules, {'setuptools': setuptools}):
                runpy.run_path(str(package / 'setup.py'))
        finally:
            os.chdir(previous)
        destination = os.path.join('share', 'py_amr_ttb', 'launch')
        launches = dict(captured['data_files'])[destination]
        installed = {Path(path).name for path in launches}
        for name in ('go_to_goal', 'square', 'star', 'circle', 'heart'):
            self.assertIn(f'lab2_{name}.launch.py', installed)
        self.assertEqual(len(installed), 12)
        self.assertNotIn('lab2_shape.launch.py', installed)
        scripts = captured['entry_points']['console_scripts']
        self.assertIn('lab2_star = py_amr_ttb.lab2.lab2_star:main', scripts)
        self.assertIn('lab2_heart = py_amr_ttb.lab2.lab2_heart:main', scripts)
        self.assertFalse(any('lab2_shape' in script for script in scripts))
        for path in launches:
            self.assertTrue((package / path).is_file())
        # Validate target modules without importing a ROS runtime.
        for script in scripts:
            target, function = script.split(' = ')[1].split(':')
            tree = ast.parse((package / Path(*target.split('.'))).with_suffix('.py').read_text())
            self.assertIn(function, [node.name for node in tree.body
                                     if isinstance(node, ast.FunctionDef)])


if __name__ == '__main__':
    unittest.main()
