"""Diagnosing a run needs measured speeds and decision reasons, but
disk writes must not delay control.

Queue CSV samples for a background writer. Bound memory, count dropped samples and
retain write errors for reporting.
"""

import csv
import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from queue import Queue, Empty, Full
from threading import Event, Thread
import time


FIELDS = ('time', 'x', 'y', 'yaw', 'measured_v', 'measured_w', 'command_v', 'command_w',
          'mode', 'reason', 'plan_age', 'worker_seconds', 'control_seconds',
          'position_sigma', 'heading_sigma', 'diagnostics')


class Telemetry:
    """A slow disk can otherwise accumulate unlimited logging work or
    block velocity updates.

    Own one CSV stream and a bounded diagnostic queue.
    """

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
        """A full logging queue must not stall the control callback.

        Try a nonblocking enqueue and count dropped samples when full.
        """
        try:
            self.queue.put_nowait(row)
        except Full:
            self.dropped += 1

    def _write(self):
        """File writes and failures need to be handled outside the
        time-sensitive control thread.

        Drain queued rows, flush when idle, and retain an error message on failure.
        After a stop request, finish pending rows while the thread remains running.
        """
        try:
            with self.stream:
                writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
                writer.writeheader()
                while not self.stop.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=.1)
                        if isinstance(item, tuple):
                            path, snapshot = item
                            def encode(value):
                                return asdict(value) if is_dataclass(value) else value.tolist()
                            path.write_text(json.dumps(snapshot, default=encode), encoding='utf-8')
                        else:
                            writer.writerow(item)
                    except Empty:
                        self.stream.flush()
        except Exception as error:
            self.error = str(error)

    def capture(self, reason, snapshot):
        """Queue the exact rejected inputs alongside this run's CSV."""
        path = self.path.with_name(self.path.stem+'-'+reason+'.json')
        try:
            self.queue.put_nowait((path, snapshot))
            return path
        except Full:
            self.dropped += 1
            return None

    def close(self):
        """Queued final samples should get a chance to reach disk
        without allowing logging to hang shutdown.

        Request a drain and wait up to two seconds.
        """
        self.stop.set()
        self.thread.join(timeout=2.)
