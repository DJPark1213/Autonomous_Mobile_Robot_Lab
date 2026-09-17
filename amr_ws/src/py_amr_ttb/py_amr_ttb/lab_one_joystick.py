import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from py_amr_ttb.lab_config import LabConfig

class TTBController(Node):

    def __init__(self):
        super().__init__('lab_one_joystick')
        # Robot topics and joystick settings are defined in LabConfig.

        # Set up the configured velocity topic with buffer size 10.
        self.publisher_ = self.create_publisher(
            Twist,
            LabConfig.CMD_VEL_TOPIC,
            10
        )

        # Subscribe to Joy messages on the configured joystick topic.
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscriber_ = self.create_subscription(
            Joy,
            LabConfig.JOY_TOPIC,
            self.joy_callback,
            qos_profile
        )

        # setup controller to run at 10hz (period=.1s) and call method controller_callback
        timer_period = 0.1
        self.timer   = self.create_timer(timer_period, self.controller_callback)

        # when count >=30, stop moving vehicle
        self.count = 0
        
        self.buttons= 0
        self.linear_x = 0
        self.linear_y = 0
        self.angular_z = 0
        self.axes= 0

    def controller_callback(self):
        # create msg which makes TTB speed 0.1 m/s and angular velocity 0.4rad/s
        # if count >=30 (~3 seconds), stop moving

        msg = Twist()
        if self.count < 30:
            msg.linear.x  = 0.1
            msg.angular.z = 0.4
        else:
            msg.linear.x  = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.count += 1

    def joy_callback(self, msg):
        # print angular velocity from imu message to console
        # ang_vel = msg.angular_velocity.z
        # self.get_logger().info(f'Angular velocity: {ang_vel:0.4f}')
        self.buttons = msg.buttons
        self.axes = msg.axes
        self.linear_x = self.axes[LabConfig.FORWARD_AXIS]
        self.angular_z = self.axes[LabConfig.TURN_AXIS]
        
        new_msg = Twist()
        new_msg.linear.x = LabConfig.TELEOP_LINEAR_SCALE * self.linear_x
        new_msg.angular.z = LabConfig.TELEOP_ANGULAR_SCALE * self.angular_z
        self.publisher_.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)

    ttb_controller = TTBController()

    rclpy.spin(ttb_controller)

    ttb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
