"""ROS-free regression simulator with ray-cast LiDAR and a separate robot plant."""

import argparse
from collections import Counter
from dataclasses import replace
import json
import math
from pathlib import Path
import tempfile
import time

import numpy as np

from py_amr_ttb.lab_config import Lab3Config
from .geometry import process_scan, inverse_points
from .motion import MotionController
from .types import RobotState
from .worker import NavigationEngine, Work


def rectangle(x0, y0, x1, y1):
    points = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(points[i], points[(i+1) % 4]) for i in range(4)]


def course(name):
    boundary = rectangle(-1., -2., 5., 2.)
    if name == 'straight':
        return boundary, (3., 0.)
    if name == 'obstacle':
        return boundary+rectangle(1., -.45, 1.35, .45), (3., 0.)
    if name == 'pocket':
        # A visible capped corridor ahead, with a way around its sides.
        return boundary+[((.65, -.6), (2., -.6)), ((2., -.6), (2., .6)),
                         ((2., .6), (.65, .6))], (3.4, 0.)
    if name == 'corner':
        return boundary+[((1.1, -2.), (1.1, .7)), ((1.1, .7), (2.3, .7))], (3.4, 0.)
    raise ValueError(name)


def raycast(walls, pose, count=360, maximum=6.):
    angles = np.linspace(-math.pi, math.pi, count, endpoint=False)+pose[2]
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    segments = np.asarray(walls, dtype=float)
    a, segment = segments[:, 0], segments[:, 1]-segments[:, 0]
    offset = a-np.asarray(pose[:2])
    cross = directions[:, 0, None]*segment[None, :, 1]-directions[:, 1, None]*segment[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (offset[:, 0]*segment[:, 1]-offset[:, 1]*segment[:, 0])[None, :]/cross
        u = (offset[None, :, 0]*directions[:, 1, None]-offset[None, :, 1]*directions[:, 0, None])/cross
    valid = (abs(cross) > 1e-10) & (t >= .03) & (u >= 0) & (u <= 1)
    distances = np.where(valid, t, np.inf).min(axis=1)
    return np.where(distances <= maximum, distances, np.inf)


def wall_distance(walls, position):
    walls = np.asarray(walls)
    start, delta = walls[:, 0], walls[:, 1]-walls[:, 0]
    alpha = np.clip(np.sum((np.asarray(position)-start)*delta, axis=1)/np.sum(delta*delta, axis=1), 0, 1)
    return float(np.linalg.norm(start+alpha[:, None]*delta-position, axis=1).min())


def plant(pose, v, w, command, dt):
    """Independent response: slower acceleration and finite braking, 5 ms substeps."""
    x, y, yaw = pose
    for _ in range(10):
        h = dt/10
        av = .35 if abs(command.v) > abs(v) and command.v*v >= 0 else .60
        aw = 1.50 if abs(command.w) > abs(w) and command.w*w >= 0 else 2.50
        v += max(-av*h, min(av*h, command.v-v))
        w += max(-aw*h, min(aw*h, command.w-w))
        yaw += w*h
        x += v*math.cos(yaw)*h
        y += v*math.sin(yaw)*h
    return (x, y, math.atan2(math.sin(yaw), math.cos(yaw))), v, w


def simulate(name='obstacle', config=None, seconds=90., start=(0., 0., 0.)):
    walls, goal = course(name)
    config = config or Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False)
    engine = NavigationEngine(config)
    motion = MotionController(config)
    pose, v, w = start, 0., 0.
    scan = plan = None
    blocked = False
    reasons, modes = Counter(), Counter()
    samples, durations = [], []
    minimum, peak = math.inf, 0.
    done = False
    for tick in range(int(seconds/config.control_period)):
        now = tick*config.control_period
        state = RobotState(pose, v, w, now, now)
        if tick % 2 == 0:
            scan = process_scan(raycast(walls, pose), -math.pi, 2*math.pi/360, .03, 6.,
                                config, pose=pose, stamp=now, received=now, sequence=tick)
        if tick % max(1, round(config.map_period/config.control_period)) == 0:
            began = time.perf_counter()
            plan = engine.step(Work(state, scan, now, blocked))
            durations.append(time.perf_counter()-began)
        target = plan.target if plan else None
        if target is not None:
            target = replace(target, point=tuple(inverse_points([target.point], plan.transform)[0]))
        result = motion.step(state, scan, target, now, config.control_period,
                             plan.diagnostics.get('speed_cap') if plan else None)
        blocked = result.blocked
        reasons[result.reason] += 1
        modes[target.mode if target else 'NONE'] += 1
        samples.append((now, *pose, v, w, result.v, result.w))
        pose, v, w = plant(pose, v, w, result, config.control_period)
        minimum = min(minimum, wall_distance(walls, pose[:2]))
        peak = max(peak, abs(v))
        if minimum < config.robot_radius:
            raise AssertionError(f'Collision at {pose}, clearance={minimum:.3f}')
        if plan and plan.status == 'DONE' and abs(v) < .005 and abs(w) < .01:
            done = True
            break
    return dict(scenario=name, completed=done, elapsed=now, distance_to_goal=math.dist(pose[:2], goal),
                minimum_clearance=minimum, peak_speed=peak, reasons=dict(reasons), modes=dict(modes),
                worker_ms_p95=float(np.percentile(durations, 95)*1000),
                samples=samples, engine=engine)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('straight', 'obstacle', 'pocket', 'corner'), default='obstacle')
    parser.add_argument('--seconds', type=float, default=90.)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--two-runs', action='store_true')
    options = parser.parse_args(args)
    with tempfile.TemporaryDirectory() as directory:
        _, goal = course(options.scenario)
        cfg = Lab3Config(goal_x=goal[0], goal_y=goal[1], run_file=str(Path(directory)/'course.npz'),
                         alignment_enabled=False)
        first = simulate(options.scenario, cfg, options.seconds)
        reports = [first]
        if options.two_runs and first['completed']:
            reports.append(simulate(options.scenario, replace(cfg, run_mode='replay'), options.seconds))
        summary = [{key: value for key, value in report.items() if key not in ('samples', 'engine')}
                   for report in reports]
        print(json.dumps(summary, indent=2))
        if options.output:
            options.output.mkdir(parents=True, exist_ok=True)
            (options.output/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
            for index, report in enumerate(reports, 1):
                np.savetxt(options.output/f'run{index}.csv', report['samples'], delimiter=',',
                           header='time,x,y,yaw,measured_v,measured_w,command_v,command_w', comments='')
                report['engine'].memory.save(options.output/f'run{index}.npz', report['engine'].mapper.grid)
        if not all(report['completed'] for report in reports):
            raise SystemExit(1)


if __name__ == '__main__':
    main()
