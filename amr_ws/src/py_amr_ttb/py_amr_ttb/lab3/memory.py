"""Without experience, the robot can repeatedly choose the same failed
direction and cannot improve a later run from its earlier path.

Retain sampled map poses, local failure penalties and saved map evidence for
recovery and replay.
"""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np

from py_amr_ttb.odom_utils import normalize_angle
from .mapping import OccupancyMap


class RunMemory:
    """Detour selection needs failed directions, and replay needs a
    record of the completed run.

    Route rows are [x, y, yaw, time]; attempt rows are [x, y, heading, count, time].
    Coordinates use odometry for route-only runs and the map frame for map-backed replay.
    Live timestamps use monotonic time.
    """

    def __init__(self, config):
        self.cfg = config
        self.route = []
        self.attempts = []
        self.complete = False

    def record(self, pose, now):
        """Recording every stationary tick adds duplicates, but
        dropping turns would lose important route shape.

        Append a map pose and time only after enough position or heading change.
        """
        if (not self.route or math.dist(pose[:2], self.route[-1][:2]) >= self.cfg.route_spacing
                or abs(normalize_angle(pose[2]-self.route[-1][2])) >= self.cfg.route_heading_spacing):
            self.route.append((*map(float, pose), float(now)))

    def fail(self, position, heading, now):
        """A stalled choice can look just as attractive at the next
        planning step unless its failure is remembered.

        Merge nearby failures with similar headings into a capped count and
        refreshed timestamp. Other directions remain separate candidates.
        """
        for entry in self.attempts:
            if (math.dist(position, entry[:2]) < self.cfg.attempt_radius
                    and abs(normalize_angle(heading-entry[2])) < .4):
                entry[3] = min(5., entry[3]+1)
                entry[4] = now
                return
        self.attempts.append([*map(float, position), float(heading), 1., float(now)])

    def penalty(self, position, heading, now):
        """Recent failures should discourage repetition, but a
        direction may become useful again after moving away or waiting.

        Return a bounded ranking penalty that fades with distance, heading
        difference and elapsed time; it does not mark cells blocked.
        """
        result = 0.
        for x, y, direction, count, stamp in self.attempts:
            space = max(0., 1-math.dist(position, (x, y))/self.cfg.attempt_radius)
            angle = max(0., math.cos(heading-direction))**8
            age = math.exp(-max(0., now-stamp)/self.cfg.history_decay)
            result += space*angle*count*age
        return min(3., result)

    def save(self, path, grid, transform=(0., 0., 0.)):
        """An interrupted save must not replace a usable learned run
        with a partially written file.

        Write arrays and JSON metadata to a temporary NPZ, flush, then atomically
        replace path. Store map layers, route, failures, configuration, transform
        and completion status without pickle.
        """
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        metadata = dict(version=1, config=asdict(self.cfg), complete=self.complete,
                        transform=list(transform), revision=grid.revision if grid is not None else 0,
                        route_only=grid is None)
        descriptor, temporary = tempfile.mkstemp(prefix=path.name+'.', dir=path.parent)
        try:
            with os.fdopen(descriptor, 'wb') as stream:
                layers = {} if grid is None else dict(
                    evidence=grid.evidence, observed=grid.observed, last_seen=grid.last_seen)
                np.savez_compressed(stream, metadata=json.dumps(metadata), **layers,
                                    route=np.asarray(self.route, dtype=float).reshape(-1, 4),
                                    attempts=np.asarray(self.attempts, dtype=float).reshape(-1, 5))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    @classmethod
    def load(cls, path, config):
        """Saved clearance and coordinates become misleading if replay
        uses different map dimensions, footprint or goal settings.

        Reject incompatible settings and malformed arrays; load without pickle. Age
        failure timestamps because monotonic clocks do not survive restarts. Return
        (memory, grid); replay establishes its pose alignment separately.
        """
        with np.load(Path(path).expanduser(), allow_pickle=False) as data:
            metadata = json.loads(str(data['metadata']))
            if metadata['version'] != 1:
                raise ValueError('Unsupported saved-run format')
            saved = metadata['config']
            keys = ('robot_radius', 'safety_margin', 'goal_x', 'goal_y')
            if not metadata.get('route_only'):
                keys += ('grid_resolution', 'grid_width', 'grid_height',
                         'grid_origin_x', 'grid_origin_y')
            for key in keys:
                if saved[key] != getattr(config, key):
                    raise ValueError(f'Saved run has different {key}')
            grid = None if metadata.get('route_only') else OccupancyMap(config)
            memory = cls(config)
            for name in (() if grid is None else ('evidence', 'observed', 'last_seen')):
                array = data[name]
                if array.shape != grid.evidence.shape or not np.isfinite(array).all():
                    raise ValueError('Invalid saved map')
                setattr(grid, name, array.copy())
            route, attempts = data['route'], data['attempts']
            if (route.ndim != 2 or route.shape[1] != 4 or not np.isfinite(route).all()
                    or attempts.ndim != 2 or attempts.shape[1] != 5 or not np.isfinite(attempts).all()):
                raise ValueError('Invalid saved route or history')
            memory.route, memory.attempts = route.tolist(), attempts.tolist()
            # Age saved failures because monotonic timestamps are local to a run.
            for attempt in memory.attempts:
                attempt[4] = -config.history_decay
            memory.complete = bool(metadata['complete'])
            if grid is not None:
                grid.revision = int(metadata['revision'])
            return memory, grid
