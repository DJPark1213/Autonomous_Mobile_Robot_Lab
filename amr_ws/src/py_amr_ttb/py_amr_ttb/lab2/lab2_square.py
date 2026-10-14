
"""Lab 2 Task 2: Navigate four corners of a square using odometry."""

import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from py_amr_ttb.lab_config import LabConfig

"""Navigate a square using stop, turn, and go."""
class Lab2Square(Node):
    

    def __init__(self):
        super().__init__('lab2_square')

        self.config = LabConfig

        # Initial robot pose
        self.initial_x = None
        self.initial_y = None
        self.initial_yaw = None

        # Current robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Square dimensions and controller parameters
        self.side_length = 1.0
        self.position_tolerance = 0.05
        self.yaw_tolerance = 0.10
        self.max_linear_speed = 0.20
        self.max_angular_speed = 0.40
        self.linear_gain = 1.0
        self.angular_gain = 2.0

        # Waypoint tracking
        self.waypoints = []
        self.waypoint_index = 0
        self.goal_reached = False

        # Velocity publisher
        self.publisher = self.create_publisher(
            Twist,
            self.config.CMD_VEL_TOPIC,
            10,
        )

        # Odometry subscriber
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.config.ODOM_TOPIC,
            self.odom_callback,
            qos_profile_sensor_data,
        )

        # Controller timer
        self.timer = self.create_timer(
            self.config.CONTROL_PERIOD,
            self.controller_callback,
        )

        self.get_logger().info(
            'Lab 2 Task 2: Square navigation started.'
        )

    def odom_callback(self, message):
        """Read robot position and orientation from odometry."""

        position = message.pose.pose.position
        q = message.pose.pose.orientation

        # Convert quaternion to yaw angle
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z),
        )

        # Save initial position and heading
        if self.initial_x is None:
            self.initial_x = position.x
            self.initial_y = position.y
            self.initial_yaw = yaw

            # Square defined relative to initial heading
            L = self.side_length
            c = math.cos(yaw)
            s = math.sin(yaw)

            self.waypoints = [
                (L*c, L*s),
                (L*c - L*s, L*s + L*c),
                (-L*s, L*c),
                (0.0, 0.0),
            ]

        # Position relative to starting point
        self.current_x = position.x - self.initial_x
        self.current_y = position.y - self.initial_y
        self.current_yaw = yaw

    def controller_callback(self):
        """Control robot motion toward each square corner."""

        command = Twist()

        # Wait until odometry is available
        if self.initial_x is None:
            return

        # Stop after all four corners
        if self.goal_reached:
            self.publisher.publish(command)
            return

        # Current target waypoint
        target_x, target_y = self.waypoints[
            self.waypoint_index
        ]

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance = math.hypot(dx, dy)

        # Check whether the current corner was reached
        if distance <= self.position_tolerance:

            self.publisher.publish(command)

            self.get_logger().info(
                f'Corner {self.waypoint_index + 1} reached.'
            )

            self.waypoint_index += 1

            if self.waypoint_index >= len(self.waypoints):
                self.goal_reached = True
                self.get_logger().info(
                    'Square navigation completed!'
                )

            return

        # Desired direction to waypoint
        target_yaw = math.atan2(dy, dx)

        # Normalize angular error to [-pi, pi]
        yaw_error = math.atan2(
            math.sin(target_yaw - self.current_yaw),
            math.cos(target_yaw - self.current_yaw),
        )

        # Step 1: Rotate toward waypoint
        if abs(yaw_error) > self.yaw_tolerance:

            command.linear.x = 0.0

            command.angular.z = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    self.angular_gain * yaw_error,
                ),
            )

        # Step 2: Drive toward waypoint
        else:
            command.linear.x = min(
                self.max_linear_speed,
                self.linear_gain * distance,
            )

            command.angular.z = max(
                -self.max_angular_speed,
                min(
                    self.max_angular_speed,
                    self.angular_gain * yaw_error,
                ),
            )

        # Publish velocity command
        self.publisher.publish(command)


def main(args=None):
    """Start the square navigation controller."""

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    node = None

    try:
        node = Lab2Square()
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
