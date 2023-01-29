import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            '/chatter',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('%s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    listener.destory_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()