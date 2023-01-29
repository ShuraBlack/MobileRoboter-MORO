import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import math as m
import numpy as np
from my_jk_rollin_kinematics import robot


class InverseKinematic(Node):

    def __init__(self):
        super().__init__('inverse_kinematic')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tool_pose',
            self.calculate_inverse_kinematic,
            1000
        )
        self.subscription

        self.publisher_ = self.create_publisher(
            JointState,
            '/joint_states',
            1000
        )

    """
    Function should publish a Join state with
    alpha (turntable),
    beta_1 (shoulder),
    beta_2 (elbow),
    beta_3 (wrist)
    """
    def calculate_inverse_kinematic(self, msg):
        rob = robot.Robot()
        point = msg.pose.position

        limit = m.sqrt(point.x**2 + point.y**2 + (point.z - rob.b - rob.h)**2)
        if not limit <= rob.l_1 + rob.l_2 + rob.l_3:
            raise ValueError("No geometric result")

        alpha = m.atan2(point.y, point.x)

        x_f = m.sqrt(m.pow(point.x, 2) + m.pow(point.y, 2))
        z_2 = point.z - rob.b - rob.h
        x_2 = x_f - rob.l_3

        a = m.sqrt(m.pow(x_2, 2) + m.pow(z_2, 2))
        if not a <= rob.l_1 + rob.l_2:
            raise ValueError("No geometric result")

        c = (m.pow(a, 2) - m.pow(rob.l_1, 2) - m.pow(rob.l_2, 2)) / (2 * rob.l_1)
        b = -m.sqrt(np.absolute(m.pow(rob.l_2, 2) - m.pow(c, 2)))

        beta_1 = m.atan2(z_2, x_2) - m.atan2(b, (rob.l_1 + c))
        beta_2 = m.atan2(b, c)
        beta_3 = 0 - beta_1 - beta_2
        beta_1 -= m.radians(90)

        self.send_result(alpha, beta_1, beta_2, beta_3)

    def send_result(self, turntable, shoulder, elbow, wrist):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name.append('wheel_left_joint')
        msg.name.append('wheel_right_joint')
        msg.name.append('turntable')
        msg.name.append('shoulder')
        msg.name.append('elbow')
        msg.name.append('wrist')

        msg.position.append(0)
        msg.position.append(0)
        msg.position.append(turntable)
        msg.position.append(shoulder)
        msg.position.append(elbow)
        msg.position.append(wrist)

        self.publisher_.publish(msg)
        out = f"Publishing: t({turntable}), s({shoulder}), e({elbow}), w({wrist})"
        self.get_logger().info(out)


def main(args=None):
    rclpy.init(args=args)

    ik = InverseKinematic()

    rclpy.spin(ik)

    ik.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
