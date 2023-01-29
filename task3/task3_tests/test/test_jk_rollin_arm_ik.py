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
from sensor_msgs.msg import JointState

# additional imports
import numpy as np
import logging


@pytest.mark.rostest
def generate_test_description():
    # Use student's draw_circle to provide implementation invariance
    # Start with delay of 5 seconds to guarantee that jk_rollin_arm_ik and test node are ready
    draw_circle_node = Node(
        package='my_jk_rollin_kinematics',
        executable='jk_rollin_draw_circle'
    )
    kinematic_node = Node(
        package='my_jk_rollin_kinematics',
        executable='jk_rollin_arm_ik'
    )
    return (
        LaunchDescription([
            draw_circle_node,
            kinematic_node,
            ReadyToTest(),
        ]),
        {
        }
    )


class TestJkRollinArmIkNode(unittest.TestCase):
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
        self.node = rclpy.create_node('test_jk_rollin_arm_ik_node')
        self.positions_in_circle = 100

        # Create expected angles
        self.expected_alphas = np.array([0., 0.014, 0.028, 0.042, 0.056, 0.069, 0.082,
                                         0.095, 0.108, 0.12, 0.131, 0.142, 0.152, 0.162,
                                         0.171, 0.179, 0.187, 0.193, 0.199, 0.205, 0.209,
                                         0.213, 0.215, 0.217, 0.218, 0.219, 0.218, 0.217,
                                         0.214, 0.211, 0.207, 0.202, 0.197, 0.19, 0.183,
                                         0.175, 0.166, 0.157, 0.147, 0.137, 0.125, 0.114,
                                         0.101, 0.089, 0.076, 0.063, 0.049, 0.035, 0.021,
                                         0.007, -0.007, -0.021, -0.035, -0.049, -0.063, -0.076,
                                         -0.089, -0.101, -0.114, -0.125, -0.137, -0.147, -0.157,
                                         -0.166, -0.175, -0.183, -0.19, -0.197, -0.202, -0.207,
                                         -0.211, -0.214, -0.217, -0.218, -0.219, -0.218, -0.217,
                                         -0.215, -0.213, -0.209, -0.205, -0.199, -0.193, -0.187,
                                         -0.179, -0.171, -0.162, -0.152, -0.142, -0.131, -0.12,
                                         -0.108, -0.095, -0.082, -0.069, -0.056, -0.042, -0.028,
                                         -0.014, -0.])

        self.expected_beta_1s = np.array([-0.651, -0.65, -0.649, -0.646, -0.643, -0.639, -0.634,
                                          -0.629, -0.624, -0.619, -0.614, -0.609, -0.604, -0.6,
                                          -0.596, -0.593, -0.59, -0.587, -0.586, -0.584, -0.584,
                                          -0.584, -0.584, -0.585, -0.586, -0.588, -0.591, -0.594,
                                          -0.597, -0.6, -0.604, -0.608, -0.613, -0.617, -0.622,
                                          -0.627, -0.632, -0.636, -0.641, -0.646, -0.65, -0.654,
                                          -0.658, -0.661, -0.664, -0.666, -0.668, -0.67, -0.671,
                                          -0.672, -0.672, -0.671, -0.67, -0.668, -0.666, -0.664,
                                          -0.661, -0.658, -0.654, -0.65, -0.646, -0.641, -0.636,
                                          -0.632, -0.627, -0.622, -0.617, -0.613, -0.608, -0.604,
                                          -0.6, -0.597, -0.594, -0.591, -0.588, -0.586, -0.585,
                                          -0.584, -0.584, -0.584, -0.584, -0.586, -0.587, -0.59,
                                          -0.593, -0.596, -0.6, -0.604, -0.609, -0.614, -0.619,
                                          -0.624, -0.629, -0.634, -0.639, -0.643, -0.646, -0.649,
                                          -0.65, -0.651])

        self.expected_beta_2s = np.array([-0.438, -0.44, -0.446, -0.456, -0.47, -0.487, -0.507,
                                          -0.529, -0.553, -0.579, -0.607, -0.635, -0.665, -0.695,
                                          -0.725, -0.756, -0.787, -0.818, -0.849, -0.88, -0.911,
                                          -0.941, -0.97, -0.999, -1.028, -1.056, -1.083, -1.109,
                                          -1.135, -1.159, -1.183, -1.206, -1.227, -1.248, -1.268,
                                          -1.286, -1.304, -1.32, -1.335, -1.349, -1.362, -1.373,
                                          -1.384, -1.393, -1.4, -1.407, -1.412, -1.416, -1.418,
                                          -1.42, -1.42, -1.418, -1.416, -1.412, -1.407, -1.4,
                                          -1.393, -1.384, -1.373, -1.362, -1.349, -1.335, -1.32,
                                          -1.304, -1.286, -1.268, -1.248, -1.227, -1.206, -1.183,
                                          -1.159, -1.135, -1.109, -1.083, -1.056, -1.028, -0.999,
                                          -0.97, -0.941, -0.911, -0.88, -0.849, -0.818, -0.787,
                                          -0.756, -0.725, -0.695, -0.665, -0.635, -0.607, -0.579,
                                          -0.553, -0.529, -0.507, -0.487, -0.47, -0.456, -0.446,
                                          -0.44, -0.438])

        self.expected_beta_3s = np.array([-0.482, -0.48, -0.476, -0.468, -0.458, -0.445, -0.43,
                                          -0.413, -0.394, -0.373, -0.351, -0.327, -0.302, -0.276,
                                          -0.25, -0.222, -0.194, -0.165, -0.136, -0.106, -0.077,
                                          -0.047, -0.017, 0.014, 0.043, 0.073, 0.103, 0.132,
                                          0.161, 0.189, 0.216, 0.243, 0.27, 0.295, 0.319,
                                          0.343, 0.365, 0.386, 0.406, 0.424, 0.441, 0.456,
                                          0.47, 0.483, 0.493, 0.502, 0.51, 0.515, 0.519,
                                          0.521, 0.521, 0.519, 0.515, 0.51, 0.502, 0.493,
                                          0.483, 0.47, 0.456, 0.441, 0.424, 0.406, 0.386,
                                          0.365, 0.343, 0.319, 0.295, 0.27, 0.243, 0.216,
                                          0.189, 0.161, 0.132, 0.103, 0.073, 0.043, 0.014,
                                          -0.017, -0.047, -0.077, -0.106, -0.136, -0.165, -0.194,
                                          -0.222, -0.25, -0.276, -0.302, -0.327, -0.351, -0.373,
                                          -0.394, -0.413, -0.43, -0.445, -0.458, -0.468, -0.476,
                                          -0.48, -0.482])

    def tearDown(self):
        self.node.destroy_node()

    def test_circle_ik(self, launch_service, proc_output):
        joint_msgs_rec_buffer: list[JointState] = []

        sub_joints = self.node.create_subscription(
            JointState,
            'joint_states',
            lambda msg: joint_msgs_rec_buffer.append(msg),
            10
        )

        try:
            # Messages should be published with 10hz.
            # For 100 messages 10s are enough, add some extra time
            end_time = time.time() + 30
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=5)
                # Stop data collection after 100 msgs
                if len(joint_msgs_rec_buffer) == 100:
                    break

            # Check if min 100 messages were received
            self.assertGreaterEqual(
                len(joint_msgs_rec_buffer), self.positions_in_circle,
                "Not enough messages received. Please make sure that the ros2 workspace " +
                "is sourced, 'tool_pose'-messages are published with a frequency " +
                "of 10 Hz and the node 'jk_rollin_arm_ik' publishes to 'joint_states'.")

            # Use first 100 received messages
            joint_msgs_rec_buffer = joint_msgs_rec_buffer[:100]

            # We may have missed some of the first messages, rotate received messages if necessary
            first_received_alpha = np.round(
                joint_msgs_rec_buffer[0].position[2], 3)
            alpha_indices = np.where(
                self.expected_alphas == first_received_alpha)[0]

            logging.getLogger().info("First received alpha: " + str(first_received_alpha))
            self.assertGreater(len(alpha_indices), 0,
                               "First received alpha not found in values " +
                               "of expected alphas")

            first_received_beta_1 = np.round(
                joint_msgs_rec_buffer[0].position[3], 3)
            beta_1_indices = np.where(
                self.expected_beta_1s == first_received_beta_1)[0]

            logging.getLogger().info("First received beta_1: " + str(first_received_beta_1))
            self.assertGreater(len(beta_1_indices), 0,
                               "First received beta_1 not found in values of expected beta_1s")

            logging.getLogger().info("Found alpha_inideces: " + str(alpha_indices))
            logging.getLogger().info("Found beta_1_inideces: " + str(beta_1_indices))

            matching_offsets = np.intersect1d(alpha_indices, beta_1_indices)
            self.assertGreater(len(matching_offsets), 0,
                               "No matching offsets found")

            logging.getLogger().info(
                "Rotating received messages by " + str(matching_offsets[0]))
            joint_msgs_rec_buffer = np.roll(
                joint_msgs_rec_buffer, matching_offsets[0])

            # Check for correct joint states
            for i in range(self.positions_in_circle):
                self.assertEqual(
                    joint_msgs_rec_buffer[i].name[0], 'wheel_left_joint',
                    "JointState.name[0] did not match the expectation")
                self.assertEqual(
                    joint_msgs_rec_buffer[i].name[1], 'wheel_right_joint',
                    "JointState.name[1] did not match the expectation")
                self.assertEqual(joint_msgs_rec_buffer[i].name[2], 'turntable',
                                 "JointState.name[2] did not match the expectation")
                self.assertEqual(joint_msgs_rec_buffer[i].name[3], 'shoulder',
                                 "JointState.name[3] did not match the expectation")
                self.assertEqual(joint_msgs_rec_buffer[i].name[4], 'elbow',
                                 "JointState.name[4] did not match the expectation")
                self.assertEqual(joint_msgs_rec_buffer[i].name[5], 'wrist',
                                 "JointState.name[5] did not match the expectation")

                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[0], 0, 3,
                    "JointState.position[0] did not match the expectation")
                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[1], 0, 3,
                    "JointState.position[1] did not match the expectation")
                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[2], self.expected_alphas[i], 3,
                    "JointState.position[2] (alpha) did not match the expectation")
                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[3], self.expected_beta_1s[i], 3,
                    "JointState.position[3] (beta_1) did not match the expectation")
                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[4], self.expected_beta_2s[i], 3,
                    "JointState.position[4] (beta_2) did not match the expectation")
                self.assertAlmostEqual(
                    joint_msgs_rec_buffer[i].position[5], self.expected_beta_3s[i], 3,
                    "JointState.position[5] (beta_3) did not match the expectation")

        finally:
            self.node.destroy_subscription(sub_joints)
