"""Delayed sensor messages and slow planning must not cause the robot
to keep moving on obsolete information.

Validate time and frames, synchronize scans with odometry, and publish velocity from
one control callback while planning runs separately.
"""

from dataclasses import fields
import math
import multiprocessing as mp
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from py_amr_ttb.lab_config import Lab3Config
from py_amr_ttb.odom_utils import read_planar_pose
from .geometry import process_scan, compose
from .motion import MotionController
from .state import StateTracker
from .telemetry import Telemetry
from .types import MotionResult
from .worker import Work, worker_main, put_latest, take_latest


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


def laser_transform(transform):
    """An upside-down or rotated laser changes beam directions; a
    tilted scan cannot be treated as ground-plane geometry.

    Return ((x, y, yaw), angle_sign) from the laser-to-base transform. Accept
    upright or inverted horizontal mounting and reject invalid rotation or tilt.
    """
    q = transform.rotation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid laser rotation')
    x, y, z, w = (value/norm for value in (q.x, q.y, q.z, q.w))
    zx, zy, zz = 2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)
    if max(abs(zx), abs(zy)) > .03:
        raise ValueError('Laser scan plane is not horizontal')
    yaw = math.atan2(2*(x*y+z*w), 1-2*(y*y+z*z))
    t = transform.translation
    return (t.x, t.y, yaw), 1. if zz > 0 else -1.


class Lab3Node(Node):
    """Independent callbacks can receive inconsistent or stale inputs,
    so velocity output needs a single place that checks readiness.

    Own input validation, current-scan control, worker communication and explicit
    stop/save/reset services.
    """

    def __init__(self):
        super().__init__('lab3_controller')
        defaults = Lab3Config()
        values = {field.name: self.declare_parameter(field.name, getattr(defaults, field.name)).value
                  for field in fields(defaults)}
        self.cfg = Lab3Config(**values)
        self.tracker = StateTracker(self.cfg)
        self.controller = MotionController(self.cfg)
        self.scan = self.plan = None
        self.sequence = 0
        self.last_scan_stamp = float('-inf')
        self.last_tick = time.monotonic()
        self.last_submit = float('-inf')
        self.last_reason = None
        self.result = MotionResult()
        self.fault = None
        self.paused = False
        self.publisher = self.create_publisher(Twist, self.cfg.topic('cmd_vel'), 1)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.odom_subscription = self.create_subscription(
            Odometry, self.cfg.topic('odom'), self.on_odom, qos_profile_sensor_data)
        self.scan_subscription = self.create_subscription(
            LaserScan, self.cfg.topic('scan'), self.on_scan, qos_profile_sensor_data)
        self.telemetry = Telemetry(self.cfg.log_dir, self.cfg.run_mode)
        self.start_worker()
        self._controller_services = [self.create_service(Trigger, '~/'+name, callback)
                         for name, callback in (('stop', self.stop_run), ('reset', self.reset_run),
                                                ('save', self.save_run))]
        self.timer = self.create_timer(self.cfg.control_period, self.tick)
        self.get_logger().info(f'Lab 3 {self.cfg.run_mode}; telemetry: {self.telemetry.path}')

    def start_worker(self):
        """A long mapping step must not hold up sensor callbacks or
        velocity updates.

        Start a separate process with one-item work and result queues.
        """
        context = mp.get_context('spawn')
        self.incoming, self.outgoing = context.Queue(1), context.Queue(1)
        self.stopping, self.save_requested = context.Event(), context.Event()
        self.worker = context.Process(target=worker_main, args=(
            self.cfg, self.incoming, self.outgoing, self.stopping, self.save_requested), daemon=True)
        self.worker.start()

    def stop_worker(self):
        """Shutdown should allow the worker to save, but must not wait
        forever if it is stuck.

        Request exit, wait for bounded intervals, then terminate if necessary and
        report that the final save may be incomplete. Close communication queues.
        """
        self.stopping.set()
        self.worker.join(timeout=3.)
        if self.worker.is_alive():
            self.worker.terminate()
            self.worker.join(timeout=1.)
            self.get_logger().error('Worker did not exit; final save may be incomplete')
        for queue in (self.incoming, self.outgoing):
            queue.cancel_join_thread()
            queue.close()

    def source_age(self, stamp):
        """A message that arrived just now may contain a measurement
        taken too long ago to use.

        Compare stamp with the ROS clock; reject stale data or times beyond the
        small future tolerance. Return age in seconds for monotonic freshness
        accounting.
        """
        age = self.get_clock().now().nanoseconds*1e-9-stamp
        if not -.05 <= age <= self.cfg.sensor_timeout:
            raise ValueError('Sensor acquisition timestamp is stale or in the future')
        return max(0., age)

    def on_odom(self, message):
        """A changed frame, invalid pose or clock reset would
        invalidate the relationship between the robot and its existing map.

        Check source age and twist frame, then update starting-frame history.
        Invalid input latches a fault requiring reset.
        """
        now = time.monotonic()
        try:
            stamp = stamp_seconds(message.header.stamp)
            received = now-self.source_age(stamp)
            if message.child_frame_id != self.cfg.base_frame:
                raise ValueError('Odometry twist frame differs from configured base_frame')
            self.tracker.update(read_planar_pose(message), message.twist.twist.linear.x,
                                message.twist.twist.angular.z, stamp, received,
                                message.header.frame_id, message.pose.covariance)
        except ValueError as error:
            # Silently accepting a frame or clock reset would invalidate the map;
            # latch a fault until the operator explicitly starts a new run.
            self.fault = str(error)

    def on_scan(self, message):
        """A scan's arrival-time pose and mounting orientation can
        differ from what its raw beam angles imply.

        Validate acquisition time, obtain the mounting transform and synchronized
        odometry, then process the readings. Invalid input clears the current scan
        so control waits.
        """
        now = time.monotonic()
        try:
            stamp = stamp_seconds(message.header.stamp)
            received = now-self.source_age(stamp)
            if stamp <= self.last_scan_stamp:
                raise ValueError('Non-increasing laser timestamp')
            if message.header.frame_id == self.cfg.base_frame:
                transform, sign = (0., 0., 0.), 1.
            else:
                tf = self.tf.lookup_transform(self.cfg.base_frame, message.header.frame_id,
                                              Time.from_msg(message.header.stamp))
                transform, sign = laser_transform(tf.transform)
            pose = self.tracker.pose_at(stamp)
            self.sequence += 1
            scan = process_scan(
                ranges=message.ranges,
                angle_min=message.angle_min,
                angle_increment=message.angle_increment,
                range_min=message.range_min,
                range_max=message.range_max,
                config=self.cfg,
                pose=pose,
                transform=transform,
                angle_sign=sign,
                stamp=stamp,
                received=received,
                sequence=self.sequence,
            )
            self.scan, self.last_scan_stamp = scan, stamp
        except (ValueError, TransformException) as error:
            self.scan = None
            self.get_logger().warning(str(error), throttle_duration_sec=2.)

    def tick(self):
        """The robot needs fresh command checks even while a worker is
        still computing the next target.

        Take the newest plan, verify sensor freshness and run state, submit work
        without waiting, then publish the controller result. Default to zero when
        prerequisites fail and enqueue diagnostic telemetry.
        """
        started = now = time.monotonic()
        dt, self.last_tick = now-self.last_tick, now
        received_plan = take_latest(self.outgoing)
        if received_plan is not None:
            self.plan = received_plan
        state, scan, plan = self.tracker.latest, self.scan, self.plan
        result = MotionResult(reason='WAITING_FOR_SENSORS')
        # Missing prerequisites must not leave the previous motion command active.
        valid = (state is not None and scan is not None and self.tracker.fresh(now)
                 and 0 <= now-scan.received <= self.cfg.sensor_timeout)
        if self.fault:
            result.reason = 'FAULT: '+self.fault
        elif self.paused:
            result.reason = 'PAUSED'
        elif not self.worker.is_alive():
            result.reason = 'WORKER_STOP: '+str(plan.diagnostics if plan else '')
        elif valid:
            if now-self.last_submit >= self.cfg.map_period or self.result.blocked:
                if put_latest(self.incoming, Work(state, scan, now, self.result.blocked)):
                    self.last_submit = now
            result = self.controller.step_plan(state, scan, plan, now, dt)
        self.result = result
        self.publish(result.v, result.w)
        if result.reason != self.last_reason:
            self.get_logger().info(result.reason)
            self.last_reason = result.reason
        if state is not None:
            pose = compose(plan.transform, state.pose) if plan else state.pose
            variance = plan.diagnostics.get('pose_variance') if plan else None
            self.telemetry.record(dict(time=now, x=pose[0], y=pose[1], yaw=pose[2],
                                       measured_v=state.v, measured_w=state.w,
                                       command_v=result.v, command_w=result.w,
                                       mode=plan.target.mode if plan and plan.target else '',
                                       reason=result.reason, plan_age=now-plan.created if plan else '',
                                       worker_seconds=plan.diagnostics.get('worker_seconds', '') if plan else '',
                                       control_seconds=time.monotonic()-started,
                                       position_sigma=math.sqrt(max(variance[:2])) if variance else '',
                                       heading_sigma=math.sqrt(variance[2]) if variance else ''))

    def publish(self, v=0., w=0.):
        message = Twist()
        message.linear.x, message.angular.z = float(v), float(w)
        self.publisher.publish(message)

    def stop_run(self, request, response):
        """An operator stop must prevent later ticks from resuming
        motion while the current run is being saved.

        Latch paused, publish zero velocity and queue saving. The response
        acknowledges the request, not disk completion; reset starts a new run.
        """
        self.paused = True
        self.publish()
        self.save_requested.set()
        response.success, response.message = True, 'Stopped; save queued. Reset starts a new run.'
        return response

    def save_run(self, request, response):
        """Disk persistence can be slow enough to delay ROS callbacks
        if performed inside the service.

        Queue the worker save request; the service does not confirm completion.
        """
        self.save_requested.set()
        response.success, response.message = True, 'Save queued; completion/errors appear in worker output.'
        return response

    def reset_run(self, request, response):
        """Changing the odometry origin while moving would invalidate
        both the saved scan coordinates and the held target.

        Require an explicit pause and near-zero measured motion. Restart worker and
        odometry tracking, discard old references and wait for fresh inputs.
        """
        state = self.tracker.latest
        if not self.paused or (state and (abs(state.v) > .01 or abs(state.w) > .03)):
            response.success, response.message = False, 'Call stop and wait until stationary before reset.'
            return response
        self.publish()
        self.stop_worker()
        self.tracker = StateTracker(self.cfg)
        self.scan = self.plan = self.fault = None
        self.last_scan_stamp = self.last_submit = float('-inf')
        self.start_worker()
        self.paused = False
        response.success, response.message = True, 'New run; waiting for fresh sensors.'
        return response

    def close(self):
        """Exiting without a final stop request or worker shutdown can
        leave commands and saved evidence unfinished.

        Publish zero, stop the worker, drain telemetry with a bounded wait and
        report logging problems.
        """
        self.publish()
        self.stop_worker()
        self.telemetry.close()
        if self.telemetry.error or self.telemetry.dropped:
            self.get_logger().warning(f'Telemetry: {self.telemetry.error}; dropped={self.telemetry.dropped}')


def main(args=None):
    """Interrupting ROS execution must still request a stop and release
    the worker and logging resources.

    Run the node with cleanup in finally, including node destruction and ROS
    shutdown.
    """
    rclpy.init(args=args)
    node = None
    try:
        node = Lab3Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
