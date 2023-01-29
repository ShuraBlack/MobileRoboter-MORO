import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

import math

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


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

    return roll_x, pitch_y, yaw_z  # in radians


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()

    def reset(self):
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0
        self.previous_error = 0

    def update(self, setpoint, current_value, dt):
        error = setpoint - current_value
        self.p_error = error
        self.i_error += error * dt
        self.d_error = (error - self.previous_error) / dt
        self.previous_error = error

        output = self.kp * self.p_error + self.ki * self.i_error + self.kd * self.d_error

        return output


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower_node")

        self.target_reached = False

        self.declare_parameter("v_p", 0.0)
        self.declare_parameter("v_max", 0.0)
        self.declare_parameter("omega_max", 0.0)
        self.declare_parameter("d_star", 0.0)
        self.declare_parameter("k_v", 1.8)
        self.declare_parameter("k_omega", 0.5)

        self.v_pid_controller = PIDController(1, 0, 0)

        self.omega_pid_controller = PIDController(1, 0, 0)

        self.v_p = self.get_parameter("v_p").get_parameter_value().double_value
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.omega_max = self.get_parameter("omega_max").get_parameter_value().double_value
        self.d_star = self.get_parameter("d_star").get_parameter_value().double_value
        self.k_v = self.get_parameter("k_v").get_parameter_value().double_value
        self.k_omega = self.get_parameter("k_omega").get_parameter_value().double_value

        self.poses = None
        self.pose_idx = 0
        self.carrot_position = None

        self.marker_publisher = self.create_publisher(Marker, "carrot", 10)

        # publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscriber get path
        self.subscription_path = self.create_subscription(
            Path,
            '/path',
            self.listener_callback_path,
            10)

    def publish_carrot(self):
        p_marker = Marker()

        p_marker.header.frame_id = "odom"
        p_marker.header.stamp = self.get_clock().now().to_msg()
        p_marker.ns = ""
        p_marker.id = 1
        p_marker.lifetime = Duration()
        p_marker.type = Marker.SPHERE
        p_marker.action = Marker.MODIFY
        p_marker.pose.position.x = self.carrot_position.x  # Position ihres vorauseilenden Punktes
        p_marker.pose.position.y = self.carrot_position.y  # Position ihres vorauseilenden Punktes
        p_marker.pose.position.z = 0.0
        p_marker.pose.orientation.x = 0.0
        p_marker.pose.orientation.y = 0.0
        p_marker.pose.orientation.z = 0.0
        p_marker.pose.orientation.w = 1.0
        p_marker.scale.x = 0.1
        p_marker.scale.y = 0.1
        p_marker.scale.z = 0.1
        p_marker.color.a = 1.0  # Don't forget to set the alpha!
        p_marker.color.r = 1.0
        p_marker.color.g = 0.5
        p_marker.color.b = 0.0

        self.marker_publisher.publish(p_marker)

    def timer_carrot(self):
        if self.pose_idx < len(self.poses) - 1:
            self.pose_idx += 1
            self.carrot_position = self.poses[self.pose_idx].pose.position
            self.publish_carrot()

    def publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def check_tolerance(self, x1, y1, x2, y2):
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        return round(distance, 2) <= self.d_star

    def listener_callback_path(self, msg):
        if self.poses is None:
            self.poses = msg.poses
            # subscriber position
            self.subscription_position = self.create_subscription(
                Odometry,
                '/odom',
                self.listener_callback_position,
                10)
            self.carrot_position = self.poses[0].pose.position
            self.publish_carrot()
            timer_period = self.v_p * 10
            self.timer_carrot = self.create_timer(timer_period, self.timer_carrot)
            # disable callback

    def drive_path(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)

    def diff(self, angle1, angle2):
        return min(abs(angle1 - angle2), math.pi - abs(angle1 - angle2))

    def listener_callback_position(self, msg):
        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation
        # current_velocity = msg.twist.twist.linear
        # current_omega = msg.twist.twist.angular

        if self.pose_idx == len(self.poses) - 1:
            if self.check_tolerance(self.carrot_position.x,
                                    self.carrot_position.y,
                                    current_position.x,
                                    current_position.y):
                self.publish_stop()
                self.target_reached = True
                return

        if not self.target_reached:
            _, _, theta = euler_from_quaternion(current_orientation.x,
                                                current_orientation.y,
                                                current_orientation.z,
                                                current_orientation.w)

            target_theta = math.atan2(self.carrot_position.y
                                      - current_position.y,
                                      self.carrot_position.x
                                      - current_position.x)

            x = math.pow(current_position.x - self.carrot_position.x, 2)
            y = math.pow(current_position.y - self.carrot_position.y, 2)
            distance = abs(math.sqrt(x + y))

            v = - self.v_pid_controller.update(self.d_star, distance, dt=1)
            omega = min(self.omega_max, self.k_omega * self.diff(target_theta, theta))

            v = min(self.v_max, self.k_v * v)

            if target_theta < 0:
                omega *= -1
            self.drive_path(v, omega)


def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    while not line_follower.target_reached:
        rclpy.spin_once(line_follower)

    rclpy.shutdown()
