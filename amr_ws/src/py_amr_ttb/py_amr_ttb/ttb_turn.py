import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu


class TTBController(Node):
    """Drive robot 10 in a short arc and report its angular velocity."""

    def __init__(self):
        super().__init__('ttb_turn')

        self.publisher = self.create_publisher(
            Twist,
            '/TTB10/cmd_vel',
            10,
        )

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            Imu,
            '/TTB10/imu',
            self.imu_callback,
            sensor_qos,
        )

        self.timer = self.create_timer(0.1, self.controller_callback)
        self.count = 0

    def controller_callback(self):
        """Publish an arc command for three seconds, then stop."""
        command = Twist()
        if self.count < 30:
            command.linear.x = 0.1
            command.angular.z = 0.4

        self.publisher.publish(command)
        self.count += 1

    def imu_callback(self, message):
        """Report the measured yaw rate."""
        angular_velocity = message.angular_velocity.z
        self.get_logger().info(
            f'Angular velocity: {angular_velocity:0.4f}',
        )


def main(args=None):
    rclpy.init(args=args)
    controller = TTBController()
    try:
        rclpy.spin(controller)
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

