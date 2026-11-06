"""Mapping/planning process, shared by ROS and deterministic simulation."""

from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full
import time

from .geometry import between, compose
from .mapping import Mapper
from .memory import RunMemory
from .planner import Planner
from .replay import optimize_route
from .types import Plan


@dataclass
class Work:
    state: object
    scan: object
    now: float
    blocked: bool = False


def put_latest(queue, value):
    """Bound memory and latency; replace pending work, never block the controller."""
    try:
        queue.put_nowait(value)
        return True
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            return False  # multiprocessing feeder has not made the item visible yet
        try:
            queue.put_nowait(value)
            return True
        except Full:
            return False


def take_latest(queue):
    result = None
    try:
        while True:
            result = queue.get_nowait()
    except Empty:
        return result


class NavigationEngine:
    def __init__(self, config):
        self.cfg = config
        self.memory = RunMemory(config)
        self.mapper = Mapper(config)
        self.aligned = config.run_mode == 'learn'
        self.saved = False
        self.last_map = float('-inf')
        route = None
        if config.run_mode == 'replay':
            saved, grid = RunMemory.load(config.run_file, config)
            if not saved.complete or len(saved.route) < 2:
                raise ValueError('Replay requires a completed first run')
            route = optimize_route(saved.route, grid, config)
            self.mapper = Mapper(config, grid)
        self.planner = Planner(config, self.memory, route)

    def step(self, work):
        started = time.perf_counter()
        if not self.aligned:
            pose, quality = self.mapper.grid.align(work.scan, work.scan.pose, initial=True)
            if not quality['accepted']:
                return Plan(None, self.mapper.transform, work.now, status='ALIGNMENT_REQUIRED',
                            diagnostics=quality)
            self.mapper.transform = compose(pose, between(work.scan.pose, (0., 0., 0.)))
            self.mapper.estimator.initialize(work.scan.pose, pose)
            self.aligned = True
        if work.now-self.last_map >= self.cfg.map_period-1e-9:
            self.mapper.update(work.scan, work.state.covariance)
            self.last_map = work.now
        plan = self.planner.step(work.state, work.scan, self.mapper, work.now, work.blocked)
        plan.diagnostics = dict(plan.diagnostics, worker_seconds=time.perf_counter()-started,
                                alignment=self.mapper.alignment,
                                pose_variance=self.mapper.estimator.covariance.diagonal().tolist())
        if plan.status in ('DONE', 'TIME_LIMIT') and not self.saved:
            self.save()
            self.saved = True
        return plan

    def save(self):
        # A replay must never replace the learned course with its shorter trajectory.
        destination = Path(self.cfg.run_file).expanduser()
        if self.cfg.run_mode == 'replay':
            destination = destination.with_name(destination.stem+'.replay.npz')
        self.memory.save(destination, self.mapper.grid, self.mapper.transform)


def worker_main(config, incoming, outgoing, stopping, save_requested):
    """Fail closed: exceptions become a stop plan visible to the ROS adapter."""
    engine = None
    try:
        engine = NavigationEngine(config)
        while not stopping.is_set():
            if save_requested.is_set():
                engine.save()
                save_requested.clear()
            try:
                work = incoming.get(timeout=.1)
            except Empty:
                continue
            newer = take_latest(incoming)
            plan = engine.step(newer if newer is not None else work)
            put_latest(outgoing, plan)
    except Exception as error:
        put_latest(outgoing, Plan(None, (0., 0., 0.), time.monotonic(), status='ERROR',
                                  diagnostics={'error': f'{type(error).__name__}: {error}'}))
    finally:
        if engine is not None and engine.memory.route:
            try:
                engine.save()
            except Exception as error:
                put_latest(outgoing, Plan(None, (0., 0., 0.), time.monotonic(), status='ERROR',
                                          diagnostics={'error': f'Save failed: {error}'}))
