"""Bounded reverse-and-advance escape search in currently observed space."""

import math
from dataclasses import replace

from .geometry import scan_at_pose, observed_travel, transform_points
from .trajectory import evaluate_trajectory
from .types import Target


def escape_sequence(state, scan, config, now, transform):
    """Find a short retreat that opens a verified forward segment.

    Both segments must fit the observed footprint corridor. Execution rechecks
    every command against fresh sensing. Targets are returned in the map frame.
    """
    local = scan_at_pose(scan, state.pose)
    rear_room = observed_travel(local, math.pi, config.radius, .30, step=.025)
    for retreat in (.10, .20):
        if rear_room < retreat + .02:
            continue
        clear, _ = evaluate_trajectory(
            state, scan, -config.reverse_speed, 0., config, now)
        if not clear:
            continue
        endpoint = transform_points([(-retreat, 0.)], state.pose)[0]
        stopped = replace(state, pose=(*endpoint, state.pose[2]), v=0., w=0.)
        shifted = scan_at_pose(scan, stopped.pose)
        for degrees in (-45, 45, -90, 90):
            heading = math.radians(degrees)
            room = observed_travel(shifted, heading, config.radius, .40, step=.025)
            travel = min(.30, room-.02)
            if travel < .15:
                continue
            forward = transform_points(
                [(travel*math.cos(heading), travel*math.sin(heading))], stopped.pose)[0]
            # Require useful displacement beyond simply returning to the start.
            if math.dist(forward, state.pose[:2]) < .10:
                continue
            points = transform_points([endpoint, forward], transform)
            return [
                Target(tuple(points[0]), 'RECOVERY_REVERSE', reverse=True),
                Target(tuple(points[1]), 'RECOVERY_ADVANCE'),
            ]
    return []
