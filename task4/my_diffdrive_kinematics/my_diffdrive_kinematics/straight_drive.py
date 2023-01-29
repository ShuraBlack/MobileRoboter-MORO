import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class StraightDrive(Node):
    def __init__(self):
        super().__init__("straight_drive_node")

        self.target_reached = False

        self.declare_parameter("v", 0.0)
        self.declare_parameter("distance", 0.0)

        self.velocity = self.get_parameter("v").get_parameter_value().double_value
        self.target_distance = self.get_parameter("distance").get_parameter_value().double_value
        self.get_logger().info("v: " + str(self.velocity))
        self.get_logger().info("distance: " + str(self.target_distance))

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

        self.start_pos_x = None

    def publish_start(self):
        msg = Twist()
        msg.linear.x = self.velocity
        self.publisher_.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        current_pos_x = msg.pose.pose.position.x

        if self.start_pos_x is None:  # init start position
            self.start_pos_x = current_pos_x

        if current_pos_x >= self.start_pos_x + self.target_distance:
            self.publish_stop()
            self.target_reached = True


def main(args=None):
    rclpy.init(args=args)
    straight_drive = StraightDrive()

    while not straight_drive.target_reached:
        rclpy.spin_once(straight_drive)

    rclpy.shutdown()
