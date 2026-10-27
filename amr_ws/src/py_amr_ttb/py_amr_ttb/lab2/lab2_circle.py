        self.progress.update(self.current_x, self.current_y)
        try:
            self.measured_linear = float(message.twist.twist.linear.x)
            self.measured_angular = float(message.twist.twist.angular.z)
        except (AttributeError, TypeError, ValueError):
            self.measured_linear = self.measured_angular = float('nan')

    def warn_once(self, warning):
        """Suppress unchanged consecutive warnings."""
        if warning != self.last_warning:
            self.get_logger().warning(warning)
            self.last_warning = warning

    def controller_callback(self):
        """Advance the timed target, chase it, and send one command per tick."""
        command = Twist()
        now = time.monotonic()
        if self.started is None:
            self.publisher.publish(command)
            return
        self.speed = self.get_parameter('speed').value
        target_speed = self.get_parameter('target_speed').value
        tx, ty = self.target.advance(now, speed=target_speed)
        fresh = odometry_is_fresh(
            now, self.last_odom_time, self.config.SENSOR_TIMEOUT)
        if self.target.done and not self.goal_reached:
            self.goal_reached = True
            self.get_logger().info(
                'Circle experiment completed: target reached its time/lap limit. '
                'Robot stopped. Ctrl+C to exit.')
        if not fresh:
            self.warn_once(
                'Stale/invalid odometry: robot stopped; target clock continues.')
        else:
            self.last_warning = None
            if not self.goal_reached and self.speed > 0:
                command.linear.x, command.angular.z = pursuit_command(
                    self.current_x, self.current_y, self.current_yaw,
                    tx, ty, self.speed, self.config.NAV_MAX_LINEAR_SPEED,
                    self.config.NAV_MAX_ANGULAR_SPEED)
        self.publisher.publish(command)
        nan = float('nan')
        x, y, yaw = self.current_x, self.current_y, self.current_yaw
        sample = {
            'elapsed_s': max(0.0, now - self.started),
            'odom_x_m': self.raw_pose[0] if fresh else nan,
            'odom_y_m': self.raw_pose[1] if fresh else nan,
            'odom_yaw_rad': self.raw_pose[2] if fresh else nan,
            'x_m': x if fresh else nan, 'y_m': y if fresh else nan,
            'yaw_rad': yaw if fresh else nan,
            'target_x_m': tx, 'target_y_m': ty,
            'target_phase_rad': self.target.phase,
            'target_laps': self.target.completed_laps,
            'robot_phase_rad': self.progress.radians if fresh else nan,
            'position_error_m': math.hypot(tx - x, ty - y) if fresh else nan,
            'radial_error_m': self.path.radial_error(x, y) if fresh else nan,
            'phase_gap_rad': self.target.phase - self.progress.radians if fresh else nan,
            'linear_command_mps': command.linear.x,
            'angular_command_radps': command.angular.z,
            'measured_linear_mps': self.measured_linear if fresh else nan,
            'measured_angular_radps': self.measured_angular if fresh else nan,
            'target_speed_mps': 0.0 if self.goal_reached else self.target.speed,
            'robot_speed_mps': 0.0 if self.goal_reached else self.speed,
            'radius_m': self.path.radius, 'direction': self.path.direction,
            'lookahead_m': self.target.lookahead,
            'requested_laps': self.target.laps,
            'duration_limit_s': self.target.duration,
            'odom_age_s': (now - self.last_odom_time
                           if self.last_odom_time is not None else nan),
            'odom_fresh': int(fresh), 'done': int(self.goal_reached),
            'data_source': 'robot_odometry',
        }
        self.last_sample = sample
        self.publish_diagnostics(sample)

    def publish_diagnostics(self, sample):
        """Keep old scalar topic names; add error/target paired telemetry."""
        fields = {
            'x': 'x_m', 'y': 'y_m', 'yaw': 'yaw_rad',
            'target_x': 'target_x_m', 'target_y': 'target_y_m',
            'radial_error': 'radial_error_m', 'progress': 'robot_phase_rad',
            'done': 'done', 'linear_command': 'linear_command_mps',
            'angular_command': 'angular_command_radps',
            'position_error': 'position_error_m',
            'target_phase': 'target_phase_rad', 'target_laps': 'target_laps',
            'target_speed': 'target_speed_mps', 'robot_speed': 'robot_speed_mps',
            'phase_gap': 'phase_gap_rad', 'odom_fresh': 'odom_fresh',
            'elapsed': 'elapsed_s', 'measured_linear': 'measured_linear_mps',
            'measured_angular': 'measured_angular_radps',
        }
        for name, field in fields.items():
            self.plot_publishers[name].publish(Float64(data=float(sample[field])))
        self.plot_publishers['laps'].publish(
            Float64(data=sample['robot_phase_rad'] / math.tau))
        self.plot_publishers['distance_to_start'].publish(
            Float64(data=math.hypot(sample['x_m'], sample['y_m'])))
        serializable = {
            key: None if isinstance(value, (int, float)) and not math.isfinite(value) else value
            for key, value in sample.items()
        }
        self.sample_publisher.publish(
            String(data=json.dumps(serializable, allow_nan=False)))


def main(args=None):
    """Run circle with zero velocity on normal Ctrl+C shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Lab2Circle()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.timer.cancel()
            if rclpy.ok():
                node.publisher.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
