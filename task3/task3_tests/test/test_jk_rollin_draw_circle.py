# default imports
import time
import pytest
import unittest

# required launch imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

# ROS imports
import rclpy
from geometry_msgs.msg import PoseStamped

# additional packages import
import numpy as np
import logging


@pytest.mark.rostest
def generate_test_description():
    draw_circle_node = Node(
        package='my_jk_rollin_kinematics',
        executable='jk_rollin_draw_circle'
    )
    return (
        LaunchDescription([
            draw_circle_node,
            ReadyToTest(),
        ]),
        {
        }
    )


class TestJkRollinDrawCircleNode(unittest.TestCase):
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
        self.node = rclpy.create_node('test_jk_rollin_draw_circle_node')

        # Create x, y and z coordinates of points that are expected to be published
        self.positions_in_circle = 100
        self.expected_x = np.array([0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18,
                                    0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18, 0.18])
        self.expected_y = np.array([0., 0.003, 0.005, 0.008, 0.01, 0.012, 0.015, 0.017,
                                    0.019, 0.022, 0.024, 0.026, 0.028, 0.029, 0.031, 0.033,
                                    0.034, 0.035, 0.036, 0.037, 0.038, 0.039, 0.039, 0.04,
                                    0.04, 0.04, 0.04, 0.04, 0.039, 0.039, 0.038, 0.037,
                                    0.036, 0.035, 0.033, 0.032, 0.03, 0.029, 0.027, 0.025,
                                    0.023, 0.021, 0.018, 0.016, 0.014, 0.011, 0.009, 0.006,
                                    0.004, 0.001, -0.001, -0.004, -0.006, -0.009, -0.011, -0.014,
                                    -0.016, -0.018, -0.021, -0.023, -0.025, -0.027, -0.029, -0.03,
                                    -0.032, -0.033, -0.035, -0.036, -0.037, -0.038, -0.039, -0.039,
                                    -0.04, -0.04, -0.04, -0.04, -0.04, -0.039, -0.039, -0.038,
                                    -0.037, -0.036, -0.035, -0.034, -0.033, -0.031, -0.029, -0.028,
                                    -0.026, -0.024, -0.022, -0.019, -0.017, -0.015, -0.012, -0.01,
                                    -0.008, -0.005, -0.003, -0.])
        self.expected_z = np.array([0.18, 0.18, 0.18, 0.179, 0.179, 0.178, 0.177, 0.176, 0.175,
                                    0.174, 0.172, 0.171, 0.169, 0.167, 0.165, 0.163, 0.161, 0.159,
                                    0.157, 0.154, 0.152, 0.149, 0.147, 0.144, 0.142, 0.139, 0.137,
                                    0.134, 0.132, 0.129, 0.127, 0.125, 0.122, 0.12, 0.118, 0.116,
                                    0.114, 0.112, 0.11, 0.109, 0.107, 0.106, 0.104, 0.103, 0.102,
                                    0.102, 0.101, 0.101, 0.1, 0.1, 0.1, 0.1, 0.101, 0.101,
                                    0.102, 0.102, 0.103, 0.104, 0.106, 0.107, 0.109, 0.11, 0.112,
                                    0.114, 0.116, 0.118, 0.12, 0.122, 0.125, 0.127, 0.129, 0.132,
                                    0.134, 0.137, 0.139, 0.142, 0.144, 0.147, 0.149, 0.152, 0.154,
                                    0.157, 0.159, 0.161, 0.163, 0.165, 0.167, 0.169, 0.171, 0.172,
                                    0.174, 0.175, 0.176, 0.177, 0.178, 0.179, 0.179, 0.18, 0.18,
                                    0.18])

    def tearDown(self):
        self.node.destroy_node()

    def test_jk_rollin_draw_circle(self, launch_service, proc_output):
        pose_msgs_rec_buffer: list[PoseStamped] = []

        sub_pose = self.node.create_subscription(
            PoseStamped,
            'tool_pose',
            lambda msg: pose_msgs_rec_buffer.append(msg),
            10
        )

        try:
            # Messages should be published with 10hz.
            # For 100 messages 10s are enough, add some extra time
            end_time = time.time() + 30
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=5)
                # Stop data collection after 100 msgs
                if len(pose_msgs_rec_buffer) == self.positions_in_circle:
                    break

            # Check if min 100 messages were received
            self.assertGreaterEqual(
                len(pose_msgs_rec_buffer), self.positions_in_circle,
                "Not enough messages received. Please make sure that the ros2 workspace " +
                "is sourced and messages are published to the topic 'tool_pose' " +
                "with a frequency of 10 Hz by the node 'jk_rollin_draw_circle'.")

            # Use first 100 received messages
            pose_msgs_rec_buffer = pose_msgs_rec_buffer[:self.positions_in_circle]

            # We may have missed some of the first messages, rotate received messages if necessary
            first_received_y = np.round(
                pose_msgs_rec_buffer[0].pose.position.y, 3)
            y_indices = np.where(self.expected_y == first_received_y)[0]

            logging.getLogger().info("First received y: " + str(first_received_y))
            self.assertGreater(len(y_indices), 0,
                               "First received position.y not found in values of expected ys")

            first_received_z = np.round(
                pose_msgs_rec_buffer[0].pose.position.z, 3)
            z_indices = np.where(self.expected_z == first_received_z)[0]

            logging.getLogger().info("First received z: " + str(first_received_z))
            self.assertGreater(len(z_indices), 0,
                               "First received position.z not found in values of expected zs")

            logging.getLogger().info("Found y_inideces: " + str(y_indices))
            logging.getLogger().info("Found z_inideces: " + str(z_indices))

            matching_offsets = np.intersect1d(y_indices, z_indices)
            self.assertGreater(len(matching_offsets), 0,
                               "No matching offsets found")

            logging.getLogger().info(
                "Rotating received messages by " + str(matching_offsets[0]))
            pose_msgs_rec_buffer = np.roll(pose_msgs_rec_buffer, matching_offsets[0])

            # Check for correct points
            for i in range(self.positions_in_circle):
                self.assertAlmostEqual(
                    pose_msgs_rec_buffer[i].pose.position.x, self.expected_x[i], 3,
                    "position.x did not match expected x")
                self.assertAlmostEqual(
                    pose_msgs_rec_buffer[i].pose.position.y, self.expected_y[i], 3,
                    "position.y did not match expected y")
                self.assertAlmostEqual(
                    pose_msgs_rec_buffer[i].pose.position.z, self.expected_z[i], 3,
                    "position.z did not match expected z")

        finally:
            self.node.destroy_subscription(sub_pose)
