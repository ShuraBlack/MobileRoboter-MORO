# default imports
import pytest
import unittest

# required launch imports
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

# ROS imports
import rclpy
from std_msgs.msg import String


@pytest.mark.rostest
def generate_test_description():
    talker_node = Node(
        package='py_pubsub',
        executable='talker'
    )
    return (
        LaunchDescription([
            talker_node,
            ReadyToTest(),
        ]),
        {
        }
    )


class TestChatterNode(unittest.TestCase):
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
        self.node = rclpy.create_node('test_chatter_node')

    def tearDown(self):
        self.node.destroy_node()

    def test_talker(self, launch_service, proc_output):
        talker_rec_buf: list[String] = []

        sub_talker = self.node.create_subscription(
            String,
            'chatter',
            lambda msg: talker_rec_buf.append(msg),
            1
        )

        try:
            # wait for one message for 60 seconds
            rclpy.spin_once(self.node, timeout_sec=60)

            # test if message was received
            self.assertEqual(len(talker_rec_buf), 1,
                             "No message received. Please make sure that the ros2 workspace " +
                             "is sourced and messages are published to the topic 'chatter'.")

        finally:
            self.node.destroy_subscription(sub_talker)
