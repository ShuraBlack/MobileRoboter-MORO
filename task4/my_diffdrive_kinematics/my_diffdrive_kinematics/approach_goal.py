import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math


# Implementierung für das Umwandeln eines quaternion zu Euler Winkel
# Die Rückgabe erfolgt in radians
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class ApproachGoal(Node):
    def __init__(self):
        super().__init__("approach_goal_node")

        self.target_reached = False

        self.declare_parameter("x", 1.5)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("tol", 0.1)
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("omega_max", 2.5)
        self.declare_parameter("k_v", 1.225)
        self.declare_parameter("k_omega", 1.225)

        self.target_pos_x = self.get_parameter("x").get_parameter_value().double_value
        self.target_pos_y = self.get_parameter("y").get_parameter_value().double_value
        self.tolerance = self.get_parameter("tol").get_parameter_value().double_value
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.omega_max = self.get_parameter("omega_max").get_parameter_value().double_value
        self.k_v = self.get_parameter("k_v").get_parameter_value().double_value
        self.k_omega = self.get_parameter("k_omega").get_parameter_value().double_value

        # publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 200)

        # subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription

    def publish_start(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def diff(self, angle1, angle2):
        return min(abs(angle1 - angle2), math.pi - abs(angle1 - angle2))

    def check_tolerance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return distance <= self.tolerance

    def listener_callback(self, msg):

        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation

        _, _, theta = euler_from_quaternion(current_orientation.x,
                                            current_orientation.y,
                                            current_orientation.z,
                                            current_orientation.w)

        pos_x = current_position.x - self.target_pos_x
        pos_y = current_position.y - self.target_pos_y
        v = min(self.v_max, self.k_v * math.sqrt(math.pow(pos_x, 2) + math.pow(pos_y, 2)))
        y = self.target_pos_y - current_position.y
        x = self.target_pos_x - current_position.x
        target_theta = math.atan2(y, x)
        omega = min(self.omega_max, self.k_omega * self.diff(target_theta, theta))

        if target_theta < 0:
            omega *= -1

        if self.check_tolerance(self.target_pos_x,
                                self.target_pos_y,
                                current_position.x,
                                current_position.y):
            self.publish_stop()
            self.target_reached = True
        else:
            self.publish_start(v, omega)


def main(args=None):
    rclpy.init(args=args)
    approach_goal = ApproachGoal()

    while not approach_goal.target_reached:
        rclpy.spin_once(approach_goal)

    rclpy.shutdown()
