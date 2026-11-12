"""The first successful run may contain exploration loops and slow
corners that need not be repeated.

Use saved map evidence to verify shortcuts and rounded corners, then follow the route with live trajectory checks.
"""

import math
import numpy as np


def route_time(points, config):
    """The shortest route can still be slow if it requires many sharp
    turns.

    Estimate seconds from linear distance and angular response limits for
    comparison. points contains map [x, y] rows. This ranking estimate does not
    guarantee execution time.
    """
    points = np.asarray(points)
    if len(points) < 2:
        return 0.
    delta = np.diff(points, axis=0)
    length = np.linalg.norm(delta, axis=1)
    headings = np.arctan2(delta[:, 1], delta[:, 0])
    turns = np.abs((np.diff(headings)+math.pi) % (2*math.pi)-math.pi)
    # Short turns may finish before reaching max angular speed, so assuming
    # every turn runs at that speed would underestimate their cost.
    turn_time = np.where(turns < config.max_angular**2/config.angular_acceleration,
                         2*np.sqrt(turns/config.angular_acceleration),
                         turns/config.max_angular+config.max_angular/config.angular_acceleration)
    return float(length.sum()/config.max_linear+turn_time.sum())


def optimize_route(route, grid, config):
    """Following every recorded turn repeats detours, but taking
    unchecked shortcuts can cross walls or unknown space.

    Try small observed-space repairs for marginal recorded corners, compare bounded
    shortcut candidates by time, and validate sampled rounding segments. Return map
    [x, y] rows. Raise ValueError if required fallback segments are not verified.
    """
    points = [tuple(row[:2]) for row in route]
    if len(points) < 2:
        raise ValueError('Saved route has fewer than two points')
    # A discretized map can invalidate a recorded corner that was barely clear.
    # Repair only within a small observed neighborhood; never trust the trace alone.
    repaired = []
    for index, point in enumerate(points):
        candidate = point
        if index not in (0, len(points)-1) and not grid.segment_clear(point, point, margin=config.route_margin):
            found = []
            for distance in (.05, .10, .15, .20):
                for angle in np.linspace(-math.pi, math.pi, 16, endpoint=False):
                    q = (point[0]+distance*math.cos(angle), point[1]+distance*math.sin(angle))
                    if (grid.segment_clear(q, q, margin=config.route_margin)
                            and (not repaired or grid.segment_clear(repaired[-1], q))):
                        found.append(q)
                if found:
                    candidate = min(found, key=lambda q: math.dist(q, points[min(index+1, len(points)-1)]))
                    break
        repaired.append(candidate)
    points = repaired
    # Dense duplicate points make shortcut search expensive, but retaining
    # spatial samples is still necessary around narrow corners.
    reduced = [points[0]]
    for p in points[1:-1]:
        if math.dist(p, reduced[-1]) >= config.route_spacing:
            reduced.append(p)
    reduced.append(points[-1])
    points = reduced
    result, i = [points[0]], 0
    while i < len(points)-1:
        best, saving = i+1, 0.
        # Long traces must not create an unbounded shortcut search; sample both
        # near and far destinations within a fixed candidate budget.
        candidates = sorted(set(np.linspace(i+1, len(points)-1, min(40, len(points)-i-1), dtype=int)), reverse=True)
        for j in candidates:
            if not grid.segment_clear(points[i], points[j], margin=config.route_margin):
                continue
            before = points[max(0, i-1):min(len(points), j+2)]
            after = ([points[i-1]] if i else []) + [points[i], points[j]]
            if j+1 < len(points):
                after.append(points[j+1])
            gain = route_time(before, config)-route_time(after, config)
            if gain > saving+1e-6:
                best, saving = j, gain
        if not grid.segment_clear(points[i], points[best]):
            raise ValueError('Recorded route is not verified free in saved map')
        result.append(points[best])
        i = best
    # Rounding a clear sharp turn can cut its inside corner into a wall.
    # Validate the rounded chain before replacing the original turn.
    smooth = [result[0]]
    for a, b, c in zip(result, result[1:], result[2:]):
        a, b, c = map(np.asarray, (a, b, c))
        trim = min(.35, np.linalg.norm(b-a)*.3, np.linalg.norm(c-b)*.3)
        if trim < .03:
            smooth.append(tuple(b))
            continue
        enter = b+(a-b)*trim/np.linalg.norm(a-b)
        leave = b+(c-b)*trim/np.linalg.norm(c-b)
        n = max(4, math.ceil(2*trim/(config.grid_resolution/2)))
        curve = [(1-t)**2*enter+2*(1-t)*t*b+t*t*leave for t in np.linspace(0, 1, n)]
        chain = [smooth[-1], *curve, tuple(c)]
        if all(grid.segment_clear(p, q, margin=config.route_margin) for p, q in zip(chain, chain[1:])):
            smooth.extend(tuple(p) for p in curve)
        else:
            smooth.append(tuple(b))
    smooth.append(result[-1])
    return np.asarray(smooth)
