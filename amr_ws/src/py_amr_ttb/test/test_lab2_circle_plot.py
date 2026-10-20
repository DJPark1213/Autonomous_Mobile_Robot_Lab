"""Exercise the independent circle plotting CLI with synthetic test fixtures."""

import csv
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class CirclePlotTests(unittest.TestCase):
    """Check geometry, progress, bad input and preservation of previous results."""

    def run_cli(self, *args):
        environment = dict(os.environ)
        source = str(Path(__file__).resolve().parents[1])
        environment['PYTHONPATH'] = source + os.pathsep + environment.get('PYTHONPATH', '')
        return subprocess.run(
            [sys.executable, '-B', '-m', 'py_amr_ttb.lab2.circle_plot', *map(str, args)],
            env=environment, capture_output=True, text=True, encoding='utf-8')

    def fixture(self, path, rows):
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('elapsed_s', 'x_m', 'y_m'))
            writer.writerows(rows)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_circle_geometry_progress_and_no_overwriting(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'circle.csv'
            output = Path(folder) / 'circle.png'
            self.fixture(source, [(i * .1, .5 * math.sin(i * math.tau / 360),
                                   -.5 * (1 - math.cos(i * math.tau / 360)))
                                  for i in range(361)])
            args = ('--csv', source, '--radius', '.5', '--direction', '-1', '--output', output)
            result = self.run_cli(*args)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output.read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
            summary = json.loads(output.with_suffix('.json').read_text())
            self.assertAlmostEqual(summary['radial_rmse_m'], 0)
            self.assertAlmostEqual(summary['directed_laps_from_odometry'], 1)
            self.assertAlmostEqual(summary['closure_error_m'], 0)
            original = output.read_bytes()
            self.assertNotEqual(self.run_cli(*args).returncode, 0)
            self.assertEqual(output.read_bytes(), original)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_empty_and_invalid_data_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'bad.csv'
            output = Path(folder) / 'bad.png'
            for rows in ([], [(0, 0, 0), (1, float('nan'), 1)]):
                self.fixture(source, rows)
                result = self.run_cli('--csv', source, '--output', output)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())

    def test_invalid_duration_rejected_before_ros_import(self):
        result = self.run_cli('--record', 'unused.csv', '--duration', '-1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--duration must be finite and positive', result.stderr)


if __name__ == '__main__':
    unittest.main()
