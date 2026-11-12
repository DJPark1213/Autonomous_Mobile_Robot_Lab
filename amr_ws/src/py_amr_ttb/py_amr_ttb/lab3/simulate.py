"""Navigation changes need repeatable checks before hardware runs,
including whether commanded motion actually reaches the goal without contact.

Feed ray-cast scans into production navigation and use an independent finite-
response plant. Report completion, clearance and timing. This simplified model omits
drift, noise and moving obstacles.
"""

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
from .geometry import process_scan
from .motion import MotionController
from .types import RobotState
from .worker import NavigationEngine, Work


def rectangle(x0, y0, x1, y1):
    points = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(points[i], points[(i+1) % 4]) for i in range(4)]


def course(name):
    """Straight travel alone does not exercise obstacle avoidance,
    capped-pocket recovery or turning around corners.

    Return repeatable wall segments and a goal in metres for the selected scenario.
    """
    if name in ('photo_slalom', 'photo_switchback'):
        return photo_course(name)
    if name == 'example_course':
        return example_course()
    if name in ('maze_s_bend', 'maze_chicane', 'maze_offset_gate', 'maze_double_bend'):
        return maze_course(name)
    boundary = rectangle(-1., -2., 5., 2.)
    if name == 'straight':
        return boundary, (3., 0.)
    if name == 'obstacle':
        return boundary+rectangle(1., -.45, 1.35, .45), (3., 0.)
    if name == 'corridor':
        return [((-.5, -.21), (3., -.21)), ((-.5, .21), (3., .21))], (2., 0.)
    if name == 'staggered':
        return [((-.5, -.65), (3.5, -.65)), ((-.5, .65), (3.5, .65)),
                ((.9, -.65), (.9, .08)), ((1.9, -.08), (1.9, .65))], (3., 0.)
    if name == 'pocket':
        # This layout tests leaving a tempting dead end to find a route around it.
        return boundary+[((.65, -.6), (2., -.6)), ((2., -.6), (2., .6)),
                         ((2., .6), (.65, .6))], (3.4, 0.)
    if name == 'corner':
        return boundary+[((1.1, -2.), (1.1, .7)), ((1.1, .7), (2.3, .7))], (3.4, 0.)
    if name in ('diamond40', 'diamond42', 'extruded_wall42'):
        return passage_course('diamond' if name.startswith('diamond') else 'extruded_wall',
                              width=.40 if name == 'diamond40' else .42)
    raise ValueError(name)



def photo_course(name):
    """Approximate the two photographed cardboard courses in metres.

    Start at (0, 0), facing +x toward the far end of the photographs.
    Dimensions and start/goal locations are assumed, pending room measurements.
    Floor tape is guidance for the human operator and supplies no obstacle returns.
    """
    if name == 'photo_slalom':
        outlines = [
            [(-.5, 1.05), (1.2, 1.05), (2.6, .95), (4.2, .8), (5.8, .85)],
            [(-.5, -1.05), (1.1, -.95), (2.6, -.9), (4.1, -.8), (5.8, -.85)],
            [(1.9, .25), (2.35, .35), (2.2, -.15), (1.9, .25)],
            [(3.65, -.25), (4.25, -.35), (4.45, .2), (3.9, .4), (3.65, -.25)],
            [(5.8, -.85), (5.8, -.35)],
            [(5.8, .35), (5.8, .85)],
        ]
        goal = (6.1, 0.)
    elif name == 'photo_switchback':
        outlines = [
            [(-.5, -1.05), (-.5, 1.05), (.6, 1.05), (1.7, 1.5),
             (2.8, 1.1), (3.9, .6), (5., .95), (6.3, .8)],
            [(-.5, -1.05), (.6, -1.05), (1.7, -.6), (2.8, -1.),
             (3.9, -1.5), (5., -.95), (6.3, -.8)],
            [(.85, .2), (1.2, .65), (1.55, .2), (1.2, -.25), (.85, .2)],
            [(2.25, -.2), (2.6, .25), (2.95, -.2), (2.6, -.65), (2.25, -.2)],
            [(3.5, -.45), (3.85, -.05), (4.2, -.45), (3.85, -.85), (3.5, -.45)],
            [(4.8, -.25), (5.3, -.3), (5.45, .15), (5., .3), (4.8, -.25)],
            [(6.3, -.8), (6.3, -.35)],
            [(6.3, .35), (6.3, .8)],
        ]
        goal = (6.6, 0.)
    else:
        raise ValueError(name)
    walls = [(a, b) for outline in outlines for a, b in zip(outline, outline[1:])]
    return walls, goal


def example_course():
    """Use the approved angular black-line checkpoint drawing as the boundary.

    Vertices are traced in its 1033 x 1523 image coordinates. The assumed
    4 mm/pixel scale is not surveyed. Checkpoint guides are not walls; +x points
    toward the top of the drawing and +y toward its left.
    """
    outlines = [
        # Left free-space boundary, including attached protrusions.
        [(464,1446),(511,1366),(502,1320),(487,1308),(456,1298),
         (454,1264),(482,1223),(463,1202),(422,1195),(389,1167),
         (389,1120),(331,1070),(331,1025),(298,994),(257,994),
         (221,1036),(202,1035),(184,1017),(185,841),
         (209,848),(242,879),(300,879),(358,823),(358,789),
         (301,789),(268,758),(223,751),(219,792),(185,841),
         (173,787),(176,697),(199,628),(198,494),(235,531),
         (305,531),(379,578),(401,602),(403,648),(434,678),
         (513,678),(513,625),(573,625),(593,602),(593,570),
         (543,515),(471,515),(474,576),(432,580),(402,557),
         (401,525),(361,484),(306,479),(320,457),(320,437),
         (366,413),(350,380),(407,335),(407,307),(389,287),
         (389,255),(420,218),(420,190),(394,164),(411,142),
         (375,101),(253,119)],
        # Right free-space boundary and lower connected zigzag.
        [(700,1446),(673,1381),(750,1381),(750,1347),(737,1335),
         (718,1337),(689,1352),(693,1244),(680,1226),(649,1221),
         (615,1158),(562,1124),(562,1091),(544,1073),(495,1053),
         (495,1022),(452,983),(452,922),(481,887),(545,887),
         (545,840),(568,811),(593,811),(617,826),(665,826),
         (729,751),(773,751),(794,736),(867,737),(890,714),
         (890,674),(830,592),(760,592),(700,530),(700,482),
         (712,460),(768,460),(829,504),(850,504),(876,482),
         (877,452),(847,303),(798,242),(733,242),(673,170),
         (582,170),(547,145),(544,78)],
        # Exterior edges close the solid wall regions shown in the drawing.
        [(253,119),(194,494),(176,697)],
        [(184,1017),(193,1180),(242,1323),(456,1298)],
        [(750,1381),(822,1381),(856,1220),(875,1000),(890,714)],
        [(890,674),(876,482),(865,300),(830,78),(544,78)],
        # The upper central island is separate from both walls.
        [(562,247),(604,236),(640,266),(654,337),(625,390),
         (563,369),(545,298),(562,247)],
    ]
    def metres(point):
        return ((1460-point[1])*.004, (595-point[0])*.004)
    walls = [(metres(a), metres(b)) for outline in outlines
             for a, b in zip(outline, outline[1:])]
    return walls, metres((475,62))



def maze_course(name):
    """Mix free-standing diamonds and wall intrusions in four fixed test layouts.

    Coordinates are metres. Start at the origin facing +x; the outer boundaries
    follow the given centreline with vertical half-width, not perpendicular width.
    """
    layouts = {
        'maze_s_bend': (
            [(-.5, 0.), (.7, 0.), (2., .6), (3.3, .3), (4.6, -.5), (6., 0.), (7., 0.)],
            1.05, [(1.5, .35, .35, .4), (4.3, -.3, .35, .4)],
            [[(3., 1.419), (3., .85), (3.25, .85), (3.25, 1.362)]], (6.7, 0.)),
        'maze_chicane': (
            [(-.5, 0.), (7., 0.)], 1.1,
            [(1.25, .15, .35, .4), (4.9, -.15, .4, .45)],
            [[(2.5, -1.1), (2.5, -.1), (2.75, -.1), (2.75, -1.1)],
             [(3.65, 1.1), (3.65, .1), (3.9, .1), (3.9, 1.1)]], (6.7, 0.)),
        'maze_offset_gate': (
            [(-.5, 0.), (2.7, 0.), (3.8, -.45), (5.3, -.45), (6.5, .15), (7., .15)],
            1., [(1.4, 0., .4, .45), (5.15, -.25, .35, .4)],
            [[(3.8, .55), (3.8, -.35), (4., -.35), (4., .55)]], (6.7, .15)),
        'maze_double_bend': (
            [(-.5, 0.), (.8, 0.), (2.2, .85), (3.3, .85), (4.7, -.55), (5.8, -.55), (7., .1)],
            1.1, [(1.7, .4, .35, .4), (3.6, .4, .35, .4), (5.6, -.4, .35, .4)],
            [[(2.7, -.25), (2.7, .25), (2.95, .25), (2.95, -.25)],
             [(6.4, .875), (6.4, .4), (6.6, .4), (6.6, .983)]], (6.7, -.06)),
    }
    centers, width, diamonds, intrusions, goal = layouts[name]
    outlines = [[(x, y+side*width) for x, y in centers] for side in (-1., 1.)]
    outlines += intrusions
    for x, y, rx, ry in diamonds:
        outlines.append([(x-rx, y), (x, y+ry), (x+rx, y), (x, y-ry), (x-rx, y)])
    return [(a, b) for outline in outlines for a, b in zip(outline, outline[1:])], goal


def passage_course(obstacle='empty', width=.42, offset=-.25):
    """Hold the passage offset fixed while varying width and obstruction."""
    if obstacle not in ('empty', 'diamond', 'extruded_wall') or width <= 0:
        raise ValueError('invalid passage scenario')
    lo, hi = offset-width/2, offset+width/2
    walls = [((-.5, -1.), (2.5, -1.)), ((-.5, 1.), (2.5, 1.)),
             ((2.5, -1.), (2.5, lo)), ((2.5, hi), (2.5, 1.)),
             ((2.5, lo), (4.3, lo)), ((2.5, hi), (4.3, hi))]
    if obstacle == 'diamond':
        vertices = [(.8, 0.), (1.3, .5), (1.8, 0.), (1.3, -.5)]
        walls += [(vertices[i], vertices[(i+1) % 4]) for i in range(4)]
    elif obstacle == 'extruded_wall':
        walls += [((1., -1.), (1., .25)), ((1., .25), (1.35, .25)),
                  ((1.35, .25), (1.35, -1.))]
    return walls, (3.8, offset)


def raycast(walls, pose, count=360, maximum=6.):
    """The navigation pipeline needs sensor-like readings even when no
    physical LiDAR is connected.

    Return nearest wall distances for count evenly spaced beams starting at -pi
    relative to pose heading. No valid hit within maximum metres returns positive
    infinity.
    """
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
    """A simulated collision must be detectable even if the
    controller's sampled laser beams miss it.

    Return the shortest distance from position to any ground-truth wall segment, for
    comparison with the physical robot radius.
    """
    walls = np.asarray(walls)
    start, delta = walls[:, 0], walls[:, 1]-walls[:, 0]
    alpha = np.clip(np.sum((np.asarray(position)-start)*delta, axis=1)/np.sum(delta*delta, axis=1), 0, 1)
    return float(np.linalg.norm(start+alpha[:, None]*delta-position, axis=1).min())


def plant(pose, v, w, command, dt):
    """Testing with instantaneous command response would hide failures
    caused by acceleration and braking delay.

    Use independent response constants and ten substeps per dt, returning pose and
    measured v, w for the next tick. At the default tick, substeps are 5 ms.
    """
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
    """Individual helper checks do not show whether sensing, planning
    and motion work together over a complete run.

    Run the repeatable loop, raise on detected body-wall overlap, and return
    completion, clearance, timing, reasons, samples and engine state. Completion
    requires a DONE plan and near-zero measured speeds.
    """
    walls, goal = course(name)
    config = config or Lab3Config(goal_x=goal[0], goal_y=goal[1], alignment_enabled=False)
    engine = NavigationEngine(config)
    motion = MotionController(config)
    pose, v, w = start, 0., 0.
    scan = plan = None
    previous_command = (0., 0.)
    blocked = False
    motion_reason = ''
    reasons, modes = Counter(), Counter()
    samples, durations, control_durations = [], [], []
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
            plan = engine.step(Work(state, scan, now, blocked, motion_reason))
            durations.append(time.perf_counter()-began)
        target = plan.target if plan else None
        began = time.perf_counter()
        result = motion.step_plan(state, scan, plan, now, config.control_period, previous_command)
        previous_command = (result.v, result.w)
        control_durations.append(time.perf_counter()-began)
        blocked = result.blocked
        motion_reason = result.reason
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
                control_ms_p95=float(np.percentile(control_durations, 95)*1000),
                samples=samples, engine=engine)


def main(args=None):
    """Regression runs need comparable results and a visible failure
    when the goal is not reached.

    Run a selected course, optionally replay a completed first run, print summaries
    and optionally export CSV/NPZ files. Incomplete runs exit with failure.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=('straight', 'obstacle', 'pocket', 'corner',
                        'corridor', 'staggered', 'diamond40', 'diamond42', 'extruded_wall42',
                        'photo_slalom', 'photo_switchback', 'example_course', 'maze_s_bend', 'maze_chicane',
                        'maze_offset_gate', 'maze_double_bend'),
                        default='obstacle')
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
                engine = report['engine']
                engine.memory.save(options.output/f'run{index}.npz',
                                   engine.mapper.grid if engine.mapper is not None else None)
        if not all(report['completed'] for report in reports):
            raise SystemExit(1)


if __name__ == '__main__':
    main()
