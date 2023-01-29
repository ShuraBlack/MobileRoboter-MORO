import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class VisualServoing(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")

        self.angle = None
        self.distance = None
        self.target_reached = False

        self.declare_parameter("tol", 0.125)
        self.declare_parameter("v_max", 0.0)
        self.declare_parameter("omega_max", 0.0)
        self.declare_parameter("k_v", 0.8)
        self.declare_parameter("k_omega", 0.8)

        self.tol = self.get_parameter("tol").get_parameter_value().double_value
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.omega_max = self.get_parameter("omega_max").get_parameter_value().double_value
        self.k_v = self.get_parameter("k_v").get_parameter_value().double_value
        self.k_omega = self.get_parameter("k_omega").get_parameter_value().double_value

        self.get_logger().info("tol: " + str(self.tol))
        self.get_logger().info("v_max: " + str(self.v_max))
        self.get_logger().info("omega_max: " + str(self.omega_max))
        self.get_logger().info("k_v: " + str(self.k_v))
        self.get_logger().info("k_omega: " + str(self.k_omega))

        # publisher
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        self.marker_publisher = self.create_publisher(Marker, "/min_distance_marker", 10)

        # subscriber
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10)

    def publishMarker(self, x, y):
        p_marker = Marker()
        p_marker.header.frame_id = "base_scan"
        p_marker.header.stamp = self.get_clock().now().to_msg()
        p_marker.ns = ""
        p_marker.id = 1
        p_marker.lifetime = Duration()
        p_marker.type = Marker.SPHERE
        p_marker.action = Marker.MODIFY
        p_marker.pose.position.x = x
        p_marker.pose.position.y = y
        p_marker.pose.position.z = 0.0
        p_marker.pose.orientation.x = 0.0
        p_marker.pose.orientation.y = 0.0
        p_marker.pose.orientation.z = 0.0
        p_marker.pose.orientation.w = 1.0
        p_marker.scale.x = 0.1
        p_marker.scale.y = 0.1
        p_marker.scale.z = 0.1
        p_marker.color.a = 1.0
        p_marker.color.r = 1.0
        p_marker.color.g = 0.5
        p_marker.color.b = 0.0

        self.marker_publisher.publish(p_marker)

    def publishStart(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.twist_publisher.publish(msg)

    def publishStop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

    def getTargetPos(self, angle, distance):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        return x, y

    def callback(self, msg):
        self.scan_param = [msg.angle_min, msg.angle_max, msg.angle_increment,
                           msg.time_increment, msg.scan_time, msg.range_min,
                           msg.range_max]
        self.results = np.array(msg.ranges)

        ang_dis = []
        for angle, distance in enumerate(self.results):
            if distance != np.inf:
                if distance < msg.range_min:
                    distance = msg.range_min
                elif distance > msg.range_max:
                    distance = msg.range_max
                ang_dis.append([angle, distance])

        if not ang_dis:
            return

        self.distance = min([ang_dis[1] for ang_dis in ang_dis])
        for angle, distance in ang_dis:
            if self.distance == distance:
                self.angle = msg.angle_min + angle * msg.angle_increment
                break

        markerX, markerY = self.getTargetPos(self.angle, self.distance)
        self.publishMarker(markerX, markerY)

        v = min(self.v_max, self.k_v * self.distance)

        omega = min(self.omega_max, self.k_omega * self.angle)

        if self.angle < 0:
            omega *= -1

        if self.distance <= self.tol:
            self.publishStop()
            self.target_reached = True
        else:
            self.publishStart(v, omega)


def main(args=None):
    rclpy.init(args=args)
    visual_servoing = VisualServoing()
    while not visual_servoing.target_reached:
        rclpy.spin_once(visual_servoing)

    rclpy.shutdown()
