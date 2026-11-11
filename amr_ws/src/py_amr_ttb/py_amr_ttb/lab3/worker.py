"""Mapping and disk writes can take longer than a control tick,
delaying the next velocity decision if run in the same callback.

Use a separate process with bounded latest-work queues. NavigationEngine also runs
directly in simulation; worker errors produce stop plans.
"""

from dataclasses import dataclass, asdict
import json
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
    """Planning needs to relate a blocked command to the sensor state
    and time used for its next decision.

    Carry state, scan, monotonic now and blocked feedback together. State and scan
    retain their own acquisition metadata.
    """

    state: object
    scan: object
    now: float
    blocked: bool = False
    motion_reason: str = ''


def put_latest(queue, value):
    """A backlog would make navigation process increasingly old scenes
    while the robot continues moving.

    Try a nonblocking enqueue, replacing a pending item if full. Return whether the
    new value was enqueued; feeder visibility or contention can cause failure.
    """
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
    """Older queued work or plans can already be obsolete when a
    consumer gets time to read them.

    Drain currently visible items and return the newest, or None without waiting.
    """
    result = None
    try:
        while True:
            result = queue.get_nowait()
    except Empty:
        return result


class NavigationEngine:
    """Planning, mapping and persistence must agree on one run's frame
    and history in both ROS and simulation.

    Own their shared lifecycle. Replay loads a completed route and requires starting
    alignment; learning creates new map evidence and memory.
    """

    def __init__(self, config):
        """A replay cannot begin from incomplete or unverifiable
        learned data.

        Load and optimize a completed saved run in replay mode; otherwise prepare a
        new learning run.
        """
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
        """Replay targets are meaningless before starting alignment,
        and later targets need an up-to-date map relationship.

        Wait for accepted initial alignment, update mapping at its configured
        period, then plan with current state and blocked feedback. Return a Plan
        with timing and uncertainty diagnostics; save once on completion or timeout.
        """
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
        plan = self.planner.step(work.state, work.scan, self.mapper, work.now, work.blocked,
                                 work.motion_reason)
        plan.diagnostics = dict(plan.diagnostics, worker_seconds=time.perf_counter()-started,
                                alignment=self.mapper.alignment,
                                pose_variance=self.mapper.estimator.covariance.diagonal().tolist())
        if plan.status in ('DONE', 'TIME_LIMIT') and not self.saved:
            self.save()
            self.saved = True
        return plan

    def save(self):
        """Saving a replay over the learned course would destroy the
        original evidence needed for future runs.

        Use a separate .replay.npz destination in replay mode.
        """
        destination = Path(self.cfg.run_file).expanduser()
        if self.cfg.run_mode == 'replay':
            destination = destination.with_name(destination.stem+'.replay.npz')
        self.memory.save(destination, self.mapper.grid, self.mapper.transform)


def worker_main(config, incoming, outgoing, stopping, save_requested):
    """A planning exception must not silently leave control following
    its last successful target.

    Consume fresh work, process save requests and publish plans until stopped. Turn
    exceptions into ERROR plans without targets, and attempt a final save when
    experience exists.
    """
    engine = None
    captured = set()
    capture_id = time.time_ns()
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
            work = newer if newer is not None else work
            plan = engine.step(work)
            failure = 'blocked_motion' if work.blocked else plan.status
            if failure in ('NO_LOCAL_ROUTE', 'blocked_motion') and failure not in captured:
                captured.add(failure)
                try:
                    directory = Path(config.log_dir).expanduser()
                    directory.mkdir(parents=True, exist_ok=True)
                    path = directory / f'failure-{capture_id}-{failure}.json'
                    payload = dict(config=asdict(config), work=asdict(work), plan=asdict(plan))
                    path.write_text(json.dumps(payload, default=lambda value: value.tolist()),
                                    encoding='utf-8')
                    plan.diagnostics['failure_capture'] = str(path)
                except Exception as error:
                    plan.diagnostics['failure_capture_error'] = str(error)
            # A plan's expiry starts when the worker produces it. Its target is
            # subsequently checked against current odometry and scans by motion
            # control. Keep input age visible separately from result freshness.
            completed = time.monotonic()
            plan.diagnostics['input_age_at_completion'] = completed-work.now
            plan.diagnostics['completed_at'] = completed
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
