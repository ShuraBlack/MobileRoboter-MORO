import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from my_jk_rollin_kinematics.transformations import rot2trans, rotx, trans
import numpy as np


class Circle (Node):

    def __init__(self):
        super().__init__('draw_circle')
        self.point = (0.180, 0.000, 0.140)
        self.iter = 0
        self.rotation = np.linspace(0, 2 * np.pi, 100)
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/tool_pose',
            1000
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.calculate_circle)

        self.calculate_circle()

    def calculate_circle(self):
        current = np.array([self.point[0], 0.000, 0.04, 1.0])
        rot = trans(np.array([0, self.point[1], self.point[2]]))\
            @ rot2trans(rotx(-self.rotation[self.iter])) @ current

        msg = PoseStamped()
        self.iter += 1
        if self.iter >= 100:
            self.iter = 0

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = rot[0]
        msg.pose.position.y = rot[1]
        msg.pose.position.z = rot[2]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    circle = Circle()

    rclpy.spin(circle)

    circle.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
