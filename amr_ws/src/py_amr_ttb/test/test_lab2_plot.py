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


if __name__ == '__main__':
    unittest.main()
