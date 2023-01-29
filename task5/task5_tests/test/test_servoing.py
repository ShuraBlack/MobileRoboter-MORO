# default imports
import time
import pytest
import unittest

# required launch imports
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ROS imports
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# additional packages import
import os.path
import math

robot_velocity_max = 0.25
robot_omega_max = 2.5
robot_tolerance = 0.15


@pytest.mark.rostest
def generate_test_description():
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch',
            'turtlebot3_pillar_world.launch.py')])
    )

    aproach_goal_node = Node(
        package='my_basic_kinematics',
        executable='visual_servoing',
        prefix=['bash -c \'sleep 20; $0 $@\''],
        # parameters=[{"v": str(robot_velocity), "distance": str(robot_target_distance)}]
        arguments=['--ros-args -p v_max:=0.25 -p omega_max:=2.5']
    )

    return (
        LaunchDescription([
            gazebo_headless,
            aproach_goal_node,
            ReadyToTest(),
        ]),
        {
        }
    )


class TestVisualServoing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('test_visual_servoing')

    def tearDown(self):
        self.node.destroy_node()

    def test_visual_servoing(self, launch_service, proc_output):
        twist_msgs_rec_buffer: list[Twist] = []
        odom_msgs_rec_buffer: list[Odometry] = []
        marker_rec_buffer: list[Marker] = []
        scan_msgs_rec_bufffer: list[LaserScan] = []

        twist_subscriber = self.node.create_subscription(
            Twist,
            'cmd_vel',
            lambda msg: twist_msgs_rec_buffer.append(msg),
            10
        )

        odom_subscriber = self.node.create_subscription(
            Odometry,
            'odom',
            lambda msg: odom_msgs_rec_buffer.append(msg),
            10
        )

        scan_subscriber = self.node.create_subscription(
            LaserScan,
            'scan',
            lambda msg: scan_msgs_rec_bufffer.append(msg),
            10
        )

        marker_subscriber = self.node.create_subscription(
            Marker,
            'min_distance_marker',
            lambda msg: marker_rec_buffer.append(msg),
            10
        )

        try:
            # Allow test to run for 60 seconds
            robot_move_start = False
            robot_move_stop = False
            robot_move_stop_position = None
            robot_move_stop_twist_msg = None
            end_time = time.time() + 120
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=120)
                if not robot_move_start:
                    if len(odom_msgs_rec_buffer) > 0 and len(twist_msgs_rec_buffer) > 0:
                        # wait for node to send drive twist msg
                        if twist_msgs_rec_buffer[-1].linear.x > 0:
                            robot_move_start = True
                    continue

                latest_twist_msg = twist_msgs_rec_buffer[-1]
                if latest_twist_msg.linear.x == 0 and latest_twist_msg.angular.z == 0:
                    # wait for node to send stop twist msg
                    robot_move_stop_position = odom_msgs_rec_buffer[-1].pose.pose.position
                    robot_move_stop_twist_msg = latest_twist_msg
                    robot_move_stop = True
                    break

                # test for correct twist msg content
                self.assertLessEqual(latest_twist_msg.linear.x, robot_velocity_max,
                                     "Robot's target linear x-velocity"
                                     " exceeds the given maximum velocity")
                self.assertEqual(latest_twist_msg.linear.y, 0,
                                 "Robot's target linear y-velocity"
                                 " does not match expected velocity")
                self.assertEqual(latest_twist_msg.linear.z, 0,
                                 "Robot's target linear z-velocity"
                                 " does not match expected velocity")

                self.assertEqual(latest_twist_msg.angular.x, 0,
                                 "Robot's target angular x-velocity"
                                 " does not match expected velocity")
                self.assertEqual(latest_twist_msg.angular.y, 0,
                                 "Robot's target angular y-velocity"
                                 " does not match expected velocity")
                self.assertLessEqual(latest_twist_msg.angular.z, robot_omega_max,
                                     "Robot's target angular z-velocity"
                                     " exceeds the given maximum velocity")

            self.assertNotEqual(len(scan_msgs_rec_bufffer), 0,
                                "Did not receive any laser scan messages."
                                "Please make sure that laser scan messages are published " +
                                "to the topic 'scan' continuously.")
            self.assertNotEqual(len(marker_rec_buffer), 0,
                                "Did not receive any marker messages."
                                "Please make sure that marker messages are published " +
                                "to the topic 'min_distance_marker'.")
            self.assertNotEqual(len(odom_msgs_rec_buffer), 0,
                                "Did not receive any odom messages."
                                "Please make sure that odom messages are published " +
                                "to the topic 'odom'.")
            self.assertNotEqual(len(twist_msgs_rec_buffer), 0,
                                "Did not receive any twist messages."
                                "Please make sure that twist messages are published " +
                                "to the topic 'cmd_vel'.")
            scan_msgs_rec_bufffer = scan_msgs_rec_bufffer[-len(
                marker_rec_buffer):]

            min_range = float('inf')
            min_index = 0

            for i in range(len(scan_msgs_rec_bufffer)):
                min_range = float('inf')
                min_index = 0
                msg = scan_msgs_rec_bufffer[i]

                for j, laser_range in enumerate(msg.ranges):
                    if laser_range < min_range:
                        min_range = laser_range
                        min_index = j
                pos_x = msg.ranges[min_index] * \
                    math.cos(msg.angle_min + min_index * msg.angle_increment)
                pos_y = msg.ranges[min_index] * \
                    math.sin(msg.angle_min + min_index * msg.angle_increment)
                min_pos_x = pos_x - robot_tolerance
                max_pos_x = pos_x + robot_tolerance
                min_pos_y = pos_y - robot_tolerance
                max_pos_y = pos_y + robot_tolerance
                self.assertTrue(min_pos_x < marker_rec_buffer[i].pose.position.x < max_pos_x,
                                "Calculated Marker position is not around the pillar")
                self.assertTrue(min_pos_y < marker_rec_buffer[i].pose.position.y < max_pos_y,
                                "Calculated Marker position is not around the pillar")
            # Test for correct node behaviour
            # Test for robot move start
            self.assertTrue(robot_move_start,
                            "Did not receive twist messages to start robot movement. " +
                            "Please make sure that twist messages are published to the topic " +
                            "'cmd_vel' continuously.")

            # Test for stop and stop message
            self.assertTrue(
                robot_move_stop, "Did not recieve any twist message to stop the robot")
            self.assertEqual(robot_move_stop_twist_msg.linear.x, 0,
                             "Robot's stop linear x-velocity does not match expected velocity")
            self.assertEqual(robot_move_stop_twist_msg.linear.y, 0,
                             "Robot's stop linear y-velocity does not match expected velocity")
            self.assertEqual(robot_move_stop_twist_msg.linear.z, 0,
                             "Robot's stop linear z-velocity does not match expected velocity")

            self.assertEqual(robot_move_stop_twist_msg.angular.x, 0,
                             "Robot's stop angular x-velocity does not match expected velocity")
            self.assertEqual(robot_move_stop_twist_msg.angular.y, 0,
                             "Robot's stop angular y-velocity does not match expected velocity")
            self.assertEqual(robot_move_stop_twist_msg.angular.z, 0,
                             "Robot's stop angular z-velocity does not match expected velocity")

            # Test for correct distance to pillar (pillar is (-1.1, 1.1) with r=0.15)
            target_tol_min_x = -1.1 - robot_tolerance - 0.15
            target_tol_max_x = -1.1 + robot_tolerance + 0.15
            target_tol_min_y = 1.1 - robot_tolerance - 0.15
            target_tol_max_y = 1.1 + robot_tolerance + 0.15
            self.assertTrue(target_tol_min_x <= robot_move_stop_position.x <= target_tol_max_x,
                            "Robot did not stop in expected x Tolerance: " +
                            str(target_tol_min_x) + " and " + str(target_tol_max_x))
            self.assertTrue(target_tol_min_y <= robot_move_stop_position.y <= target_tol_max_y,
                            "Robot did not stop in expected y Tolerance: " +
                            str(target_tol_min_y) + " and " + str(target_tol_max_y))

        finally:
            self.node.destroy_subscription(twist_subscriber)
            self.node.destroy_subscription(odom_subscriber)
            self.node.destroy_subscription(scan_subscriber)
            self.node.destroy_subscription(marker_subscriber)
