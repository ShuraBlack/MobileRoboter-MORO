import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math


class CurveDrive(Node):
    def __init__(self):
        super().__init__("curve_drive_node")

        self.start_position = None
        self.target_reached = False

        self.declare_parameter("v", 0.0)
        self.declare_parameter("r", 0.0)
        self.declare_parameter("alpha", 0.0)

        self.velocity = self.get_parameter("v").get_parameter_value().double_value
        self.radius = self.get_parameter("r").get_parameter_value().double_value
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value

        self.angular_velocity = self.velocity / self.radius

        if self.alpha != 0.0:
            self.target_distance = self.alpha * self.radius  # math.pi * 180

        # publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_start)

        # subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription

        self.publish_start()

    def publish_start(self):
        msg = Twist()

        msg.linear.x = self.velocity
        msg.angular.z = self.angular_velocity

        if self.alpha < 0:
            msg.angular.z = -1 * msg.angular.z

        self.publisher_.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        current_position = msg.pose.pose.position  # .x .y. .z

        if self.start_position is None:  # init start position
            self.start_position = current_position

        if self.target_distance is not None:
            pos_x = math.pow(current_position.x - self.start_position.x, 2)
            pos_y = math.pow(current_position.y - self.start_position.y, 2)
            current_distance = math.sqrt(pos_x + pos_y)
            if current_distance >= self.target_distance:
                self.publish_stop()
                self.target_reached = True


def main(args=None):
    rclpy.init(args=args)
    curve_drive = CurveDrive()

    while not curve_drive.target_reached:
        rclpy.spin_once(curve_drive)

    rclpy.shutdown()
