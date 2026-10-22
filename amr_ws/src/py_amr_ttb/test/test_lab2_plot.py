"""Check plotting CLI failures and actual PNG/summary output using fixture CSVs."""

import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[3] / 'scripts/plot_lab2.py'


class PlotTests(unittest.TestCase):
    """Synthetic inputs here only test the tool; they are not robot evidence."""

    def run_cli(self, *args):
        return subprocess.run([sys.executable, '-B', str(SCRIPT), *map(str, args)],
                              capture_output=True, text=True, encoding='utf-8')

    def fixture(self, path, rows):
        with path.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('elapsed_s', 'x_m', 'y_m'))
            writer.writerows(rows)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_png_summary_and_overwrite_protection(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'fixture.csv'
            output = Path(folder) / 'fixture.png'
            self.fixture(source, [(0, 0, 0), (1, .3, .4), (2, 1.5, 1.5)])
            result = self.run_cli('--csv', source, '--route', 'goal', '--output', output)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(output.read_bytes()[:8], b'\x89PNG\r\n\x1a\n')
            summary = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(summary['samples'], 3)
            self.assertEqual(summary['final_distance_to_last_waypoint_m'], 0)
            self.assertGreater(summary['measured_path_length_m'], 2)
            old = output.read_bytes()
            result = self.run_cli('--csv', source, '--output', output)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_bytes(), old)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_empty_and_nonfinite_trajectory_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'fixture.csv'
            for rows in ([], [(0, 0, 0), (1, 'nan', 1)]):
                self.fixture(source, rows)
                result = self.run_cli('--csv', source, '--output', Path(folder) / 'bad.png')
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((Path(folder) / 'bad.png').exists())

    def test_invalid_duration_rejected_before_ros_import(self):
        result = self.run_cli('--record', 'unused.csv', '--duration', '-1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--duration must be finite and positive', result.stderr)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_goal_overshoot_and_velocity_recording_plot(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'overshoot.csv'
            output = Path(folder) / 'overshoot.png'
            with source.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('elapsed_s', 'x_m', 'y_m', 'yaw_rad',
                                 'measured_linear_m_s', 'command_linear_m_s'))
                writer.writerows([
                    (0, 0, 0, 0, 0, 'nan'),
                    (1, 1.5, 1.5, .785, .1, 0),
                    (2, 1.6, 1.6, .785, 0, 0),
                ])
            result = self.run_cli('--csv', source, '--route', 'goal', '--output', output)
            self.assertEqual(result.returncode, 0, result.stderr)
            stats = json.loads(output.with_suffix('.json').read_text())
            self.assertAlmostEqual(stats['max_goal_line_overshoot_m'], 2 ** .5 * .1)
            self.assertEqual(stats['minimum_distance_to_goal_m'], 0)
            self.assertGreater(stats['final_distance_to_last_waypoint_m'], 0)

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_star_route_cli(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'star_fixture.csv'
            output = Path(folder) / 'star_fixture.png'
            self.fixture(source, [(0, 0, 0), (1, .1, .2), (2, 0, 0)])
            result = self.run_cli('--csv', source, '--route', 'star', '--output', output)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(output.is_file())

    @unittest.skipUnless(importlib.util.find_spec('matplotlib'), 'matplotlib required')
    def test_heart_route_and_width_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'heart_fixture.csv'
            output = Path(folder) / 'heart_fixture.png'
            self.fixture(source, [(0, 0, 0), (1, .3, .2), (2, 0, 0)])
            result = self.run_cli('--csv', source, '--route', 'heart', '--heart-width', '.6',
                                  '--output', output)
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads(output.with_suffix('.json').read_text())
            self.assertEqual(summary['route'], 'heart')
            self.assertEqual(summary['heart_width_m'], .6)
            self.assertEqual(summary['final_distance_to_last_waypoint_m'], 0)

    def test_invalid_heart_width_rejected(self):
        result = self.run_cli('--csv', 'unused.csv', '--route', 'heart', '--heart-width', 'nan')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('--heart-width must be finite and positive', result.stderr)


if __name__ == '__main__':
    unittest.main()
