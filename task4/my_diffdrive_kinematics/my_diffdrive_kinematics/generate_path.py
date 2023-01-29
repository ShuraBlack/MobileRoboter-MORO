import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import math


class GeneratePath(Node):
    def __init__(self):
        super().__init__("generate_path_node")

        self.finished = False

        self.declare_parameter("waypoints_x", [0.0])
        self.declare_parameter("waypoints_y", [0.0])

        waypoints_x = self.get_parameter("waypoints_x").get_parameter_value().double_array_value
        waypoints_y = self.get_parameter("waypoints_y").get_parameter_value().double_array_value
        self.get_logger().info("waypoints_x: " + str(waypoints_x))
        self.get_logger().info("waypoints_y: " + str(waypoints_y))

        # interp waypoints
        interpolated_waypoints = self.interpolate_points(waypoints_x, waypoints_y)

        # create poses
        self.poses = []
        for x, y in interpolated_waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            self.poses.append(pose)

        # publisher
        self.publisher_ = self.create_publisher(Path, '/path', 1)

        timer_period = 1  # second/s
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def interpolate_points(self, waypoints_x, waypoints_y):
        # Distance between interpolated points
        point_distance = 0.1  # meters

        # Initialize interpolated path with first point
        interpolated_path = [(waypoints_x[0], waypoints_y[0])]

        # Interpolate points between each pair of waypoints
        for i in range(len(waypoints_x) - 1):
            waypoint1 = (waypoints_x[i], waypoints_y[i])
            waypoint2 = (waypoints_x[i + 1], waypoints_y[i + 1])
            wp_x = math.pow(waypoint1[0] - waypoint2[0], 2)
            wp_y = math.pow(waypoint1[1] - waypoint2[1], 2)
            distance_between_waypoints = math.sqrt(wp_x + wp_y)
            num_points = int(distance_between_waypoints / point_distance)
            for j in range(1, num_points + 1):
                interpolated_point = (
                    waypoint1[0] + (waypoint2[0] - waypoint1[0]) * j / num_points,
                    waypoint1[1] + (waypoint2[1] - waypoint1[1]) * j / num_points
                )
                interpolated_path.append(interpolated_point)

        return interpolated_path

    def timer_callback(self):
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = self.poses
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    generate_path = GeneratePath()

    rclpy.spin(generate_path)

    generate_path.destroy_node()
    rclpy.shutdown()
