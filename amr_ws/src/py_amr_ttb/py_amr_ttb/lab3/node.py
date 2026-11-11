"""Delayed sensor messages and slow planning must not cause the robot
to keep moving on obsolete information.

Validate time and frames, synchronize scans with odometry, and publish velocity from
one control callback while planning runs separately.
"""

from dataclasses import fields, replace
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
import json
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
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from tf2_ros import Buffer, TransformListener, TransformException

from py_amr_ttb.lab_config import Lab3Config
from py_amr_ttb.odom_utils import read_planar_pose
from .geometry import process_scan, compose
from .motion import MotionController
from .state import StateTracker
from .telemetry import Telemetry
from .types import MotionResult
from .coverage import prepare_coverage
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
        self.configure_base()
        self.tracker = StateTracker(self.cfg)
        self.controller = MotionController(self.cfg)
        self.scan = self.plan = None
        self.scan_history = deque(maxlen=6)
        self.coverage_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='coverage')
        self.coverage_future = None
        self.prepared_coverage = None
        self.coverage_sequence = -1
        self.coverage_epoch = 0
        self.coverage_job_epoch = 0
        self.sequence = 0
        self.last_scan_stamp = float('-inf')
        self.last_tick = time.monotonic()
        self.last_submit = float('-inf')
        self.last_reason = None
        self.last_target = None
        self.captured_failures = set()
        self.last_diagnostic_time = float('-inf')
        self.result = MotionResult()
        self.result_target = None
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

    def find_base_parameter_node(self):
        """Discover parameter services inside this robot's namespace."""
        configured = self.declare_parameter('motion_control_node', '').value.strip()
        if configured:
            if not configured.startswith('/'):
                raise ValueError('motion_control_node must be an absolute ROS node name')
            return configured.rstrip('/')
        expected = self.cfg.topic('motion_control')
        deadline = time.monotonic()+15.
        available = []
        while rclpy.ok() and time.monotonic() < deadline:
            services = dict(self.get_service_names_and_types())
            available = sorted(
                name.removesuffix('/set_parameters') for name, types in services.items()
                if name.endswith('/set_parameters')
                and 'rcl_interfaces/srv/SetParameters' in types
                and 'rcl_interfaces/srv/GetParameters' in services.get(
                    name.removesuffix('/set_parameters')+'/get_parameters', []))
            candidates = [name for name in available
                          if name.startswith(self.cfg.robot_namespace+'/')
                          and name.endswith('/motion_control')]
            if expected in candidates:
                return expected
            if len(candidates) == 1:
                return candidates[0]
            rclpy.spin_once(self, timeout_sec=.2)
        raise RuntimeError(
            f'Cannot discover Create 3 motion_control under {self.cfg.robot_namespace}. '
            f'Visible parameter nodes: {available}. '
            'Set motion_control_node:=/actual/node/name after checking ros2 service list -t. '
            'If the base is absent, check its ROS domain and discovery configuration.')

    def configure_base(self):
        """Set the requested base mode and verify its speed before starting control."""
        mode = self.declare_parameter('safety_override', 'unchanged').value
        if mode not in ('unchanged', 'none', 'backup_only', 'full'):
            raise ValueError('safety_override must be unchanged, none, backup_only, or full')
        if mode != 'full' and self.cfg.max_linear > .306:
            raise ValueError('Speeds above 0.306 m/s require safety_override=full')
        if mode == 'unchanged':
            return
        base_node = self.find_base_parameter_node()
        self.get_logger().info(f'Configuring Create 3 through {base_node}')
        setter = self.create_client(SetParameters, base_node+'/set_parameters')
        getter = self.create_client(GetParameters, base_node+'/get_parameters')
        try:
            if not setter.wait_for_service(timeout_sec=10.) or not getter.wait_for_service(timeout_sec=10.):
                raise RuntimeError(f'Create 3 parameter services unavailable at {base_node}; check ros2 service list -t')
            request = SetParameters.Request(parameters=[
                Parameter(name='safety_override', value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING, string_value=mode))])
            future = setter.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.)
            if not future.done():
                raise RuntimeError('Timed out configuring Create 3 safety_override')
            response = future.result()
            if response is None or len(response.results) != 1 or not response.results[0].successful:
                reason = response.results[0].reason if response and response.results else 'empty response'
                raise RuntimeError('Create 3 rejected safety_override: '+reason)
            future = getter.call_async(GetParameters.Request(names=['max_speed']))
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.)
            if not future.done():
                raise RuntimeError('Timed out reading Create 3 max_speed')
            response = future.result()
            if (response is None or len(response.values) != 1
                    or response.values[0].type != ParameterType.PARAMETER_DOUBLE):
                raise RuntimeError('Create 3 returned an invalid max_speed')
            maximum = response.values[0].double_value
            if not math.isfinite(maximum) or maximum+1e-6 < self.cfg.max_linear:
                raise RuntimeError(
                    f'Create 3 max_speed {maximum} is below requested {self.cfg.max_linear}')
            self.get_logger().info(
                f'Create 3 safety_override={mode}; base limit={maximum:.3f} m/s; '
                f'navigation limit={self.cfg.max_linear:.3f} m/s')
            if mode == 'full':
                self.get_logger().warning('Create 3 cliff protection and backup limit are disabled')
        finally:
            self.destroy_client(setter)
            self.destroy_client(getter)

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

    def on_odom(self, message):
        """A changed frame, invalid pose or clock reset would
        invalidate the relationship between the robot and its existing map.

        Check source age and twist frame, then update starting-frame history.
        Invalid input latches a fault requiring reset.
        """
        now = time.monotonic()
        try:
            stamp = stamp_seconds(message.header.stamp)
            received = now
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
            received = now
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
            self.scan_history.append(scan)
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
            target_key = ((tuple(received_plan.target.point), received_plan.target.mode)
                          if received_plan.target else None)
            if target_key != self.last_target:
                self.get_logger().info('NAV_TARGET '+json.dumps(target_key))
                self.last_target = target_key
        state, scan, plan = self.tracker.latest, self.scan, self.plan
        if self.coverage_future is not None and self.coverage_future.done():
            try:
                prepared = self.coverage_future.result()
                if self.coverage_job_epoch == self.coverage_epoch:
                    self.prepared_coverage = prepared
            except Exception as error:
                self.get_logger().error('Coverage preparation failed: '+str(error))
            self.coverage_future = None
        if scan is not None and self.coverage_future is None and scan.sequence != self.coverage_sequence:
            observations = tuple(previous for previous in self.scan_history
                                 if 0 <= now-previous.received <= self.cfg.sensor_timeout)
            if observations:
                self.coverage_sequence = scan.sequence
                self.coverage_job_epoch = self.coverage_epoch
                self.coverage_future = self.coverage_pool.submit(
                    prepare_coverage, observations, bridge_small_gaps=True)
        if scan is not None:
            scan = replace(scan, coverage=self.prepared_coverage, coverage_time=now,
                           coverage_timeout=self.cfg.sensor_timeout, history=tuple(
                previous for previous in self.scan_history
                if previous.sequence != scan.sequence
                and 0 <= now-previous.received <= self.cfg.sensor_timeout))
        result = MotionResult(reason='WAITING_FOR_SENSORS')
        controller_state = dict(aligning=self.controller.aligning, settling=self.controller.settling)
        # Missing prerequisites must not leave the previous motion command active.
        valid = (state is not None and scan is not None and self.tracker.fresh(now)
                 and 0 <= now-scan.received <= self.cfg.sensor_timeout)
        if self.fault:
            result.reason = 'FAULT: '+self.fault
        elif self.paused:
            result.reason = 'PAUSED'
        elif not self.worker.is_alive():
            result.reason = 'WORKER_STOP: '+str(plan.diagnostics if plan else '')
        elif valid and (scan.coverage is None or
                        scan.coverage.select(now, self.cfg.sensor_timeout) is None):
            result.reason = 'WAITING_FOR_COVERAGE'
        elif valid:
            if now-self.last_submit >= self.cfg.map_period:
                if put_latest(self.incoming, Work(state, scan, now, self.result.blocked,
                                                  self.result.reason, self.result_target)):
                    self.last_submit = now
            result = self.controller.step_plan(state, scan, plan, now, dt)
        finished = time.monotonic()
        if finished-started > self.cfg.control_period:
            result = MotionResult(reason='CONTROL_DEADLINE', diagnostics={
                'elapsed': finished-started, 'budget': self.cfg.control_period})
        elif (result.v or result.w) and finished > result.diagnostics['valid_until']:
            result = MotionResult(reason='COMMAND_EXPIRED', diagnostics=dict(
                result.diagnostics, checked_at=finished, rejected_v=result.v, rejected_w=result.w))
        self.result = result
        self.result_target = plan.target.key if plan and plan.target else None
        self.publish(result.v, result.w)
        if result.reason in ('TRANSLATION_COVERAGE', 'TRANSLATION_OBSTACLE',
                             'EMERGENCY_STOP', 'ALIGN_BLOCKED') and result.reason not in self.captured_failures:
            path = self.telemetry.capture(result.reason, dict(
                config=self.cfg, state=state, scan=scan, plan=plan,
                now=now, dt=dt, controller_state=controller_state, result=result))
            if path is not None:
                self.captured_failures.add(result.reason)
                self.get_logger().warning('Exact motion failure queued: '+str(path))
        if result.reason != self.last_reason:
            self.get_logger().info(result.reason)
            self.last_reason = result.reason
        failure_details = {}
        if result.reason in ('NO_LOCAL_ROUTE', 'RECOVERY_STALLED', 'TURN_WITHOUT_PROGRESS',
                             'EMERGENCY_STOP', 'BRAKE_PATH_BLOCKED',
                             'WAITING_FOR_SENSORS', 'WAITING_FOR_FRESH_PLAN',
                             'CONTROL_DEADLINE', 'COMMAND_EXPIRED', 'SEARCHING_ROUTE', 'BRAKE_ROTATION',
                             'TRANSLATION_COVERAGE', 'TRANSLATION_OBSTACLE',
                             'ALIGN_BLOCKED', 'WAITING_FOR_COVERAGE'):
            failure_details = dict(
                reason=result.reason,
                odom_age=now-state.received if state else None,
                scan_age=now-scan.received if scan else None,
                plan_age=now-plan.created if plan else None,
                measured_v=state.v if state else None,
                measured_w=state.w if state else None,
                sensor_timeout=self.cfg.sensor_timeout,
                coverage_age=now-scan.coverage.receipts[0] if scan and scan.coverage else None,
                coverage_pending=self.coverage_future is not None,
                planner=plan.diagnostics if plan else {},
                clearance=result.diagnostics,
            )
            if now-self.last_diagnostic_time >= 2.:
                self.get_logger().warning('NAV_DIAGNOSTIC '+json.dumps(failure_details, default=str))
                self.last_diagnostic_time = now
        if state is not None:
            pose = compose(plan.transform, state.pose) if plan else state.pose
            variance = plan.diagnostics.get('pose_variance') if plan else None
            self.telemetry.record(dict(time=now, x=pose[0], y=pose[1], yaw=pose[2],
                                       measured_v=state.v, measured_w=state.w,
                                       command_v=result.v, command_w=result.w,
                                       mode=plan.target.mode if plan and plan.target else '',
                                       reason=result.reason,
                                       diagnostics=json.dumps(failure_details or result.diagnostics, default=str),
                                       plan_age=now-plan.created if plan else '',
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
        self.scan_history.clear()
        self.captured_failures.clear()
        self.coverage_epoch += 1
        self.prepared_coverage = None
        self.coverage_sequence = -1
        self.controller = MotionController(self.cfg)
        self.result = MotionResult()
        self.result_target = None
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
        try:
            if self.context.ok():
                self.publish()
        finally:
            self.stop_worker()
            self.telemetry.close()
            self.coverage_pool.shutdown(wait=False, cancel_futures=True)
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
