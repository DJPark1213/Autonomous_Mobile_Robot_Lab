"""Bounded, asynchronous CSV telemetry; disk I/O stays off the control callback."""

import csv
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Event, Thread
import time


FIELDS = ('time', 'x', 'y', 'yaw', 'measured_v', 'measured_w', 'command_v', 'command_w',
          'mode', 'reason', 'plan_age', 'worker_seconds', 'control_seconds',
          'position_sigma', 'heading_sigma')


class Telemetry:
    def __init__(self, directory, mode):
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f'{mode}-{time.time_ns()}.csv'
        self.queue = Queue(maxsize=2000)
        self.stop = Event()
        self.error = None
        self.dropped = 0
        # Open before driving so unwritable destinations fail during startup.
        self.stream = self.path.open('w', newline='', encoding='utf-8')
        self.thread = Thread(target=self._write, daemon=True)
        self.thread.start()

    def record(self, row):
        try:
            self.queue.put_nowait(row)
        except Full:
            self.dropped += 1

    def _write(self):
        try:
            with self.stream:
                writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
                writer.writeheader()
                while not self.stop.is_set() or not self.queue.empty():
                    try:
                        writer.writerow(self.queue.get(timeout=.1))
                    except Empty:
                        self.stream.flush()
        except Exception as error:
            self.error = str(error)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
