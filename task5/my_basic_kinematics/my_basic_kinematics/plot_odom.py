import math
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


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

    return roll_x, pitch_y, yaw_z


class PlotOdom(Node):
    def __init__(self):
        super().__init__("plot_odom_node")
        self.data = []
        self.target_reached = False
        self.last_time = None

        self.listener = self.create_subscription(Odometry, "/odom_with_noise", self.callback, 10)

    def callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        current_orientation = msg.pose.pose.orientation

        _, _, theta = euler_from_quaternion(current_orientation.x, current_orientation.y,
                                            current_orientation.z, current_orientation.w)

        covMat = msg.pose.covariance

        sigma_pose = np.array([[covMat[0], covMat[1], covMat[5]],
                               [covMat[6], covMat[7], covMat[11]],
                               [covMat[30], covMat[31], covMat[35]]])
        pose = [x, y, theta]

        self.data.append((pose, sigma_pose))

        if len(self.data) == 360:
            print('Data collected')
            self.target_reached = True
            self.draw()

    def draw(self):
        idx = range(0, 360, 10)
        for i in idx:
            pose, sigma = self.data[i]
            plot_pose_covariance(pose, sigma)

        plt.savefig('/odometrie.png', dpi=300)


# --------
# plot position and sigma ellipse
#
def plot_position_covariance(position, sigma_position, color='b'):
    assert sigma_position.shape == (2, 2), 'sigma_pose should be 2*2 matrix'
    assert len(position) == 2, 'position must be of the form [x,y]'

    # if sigma_position is close to (0,0; 0,0) there will be problems with singularity
    if sigma_position[0, 0] < 1.0E-10 or sigma_position[1, 1] < 1.0E-10:
        return
    d, V = la.eig(la.inv(sigma_position))

    # ellipse not rotated:
    gamma = np.linspace(0, 2 * math.pi, 80)
    xp = np.sin(gamma) / math.sqrt(d[0])
    yp = np.cos(gamma) / math.sqrt(d[1])

    # rotate and move ellipse
    Xp = np.vstack((xp, yp))
    Xp = V.dot(Xp)
    xp = Xp[0, :] + position[0]
    yp = Xp[1, :] + position[1]
    plt.plot(xp, yp, color + '-', linewidth=0.5)
    plt.plot(position[0], position[1], 'ko', markersize=6)


# --------
# plot position and sigma ellipse and
# plot orientation as cone.
#
def plot_pose_covariance(pose, sigma_pose, color='b'):
    assert sigma_pose.shape == (3, 3), 'sigma_pose should be 3x3 matrix'
    assert len(pose) == 3, 'pose must be of the form [x,y,theta]'

    # plot ellipse
    plot_position_covariance(pose[0:2], sigma_pose[0:2, 0:2], color)

    # plot orientation and sigma theta as cone
    theta = pose[2]
    sigma_theta = math.sqrt(sigma_pose[2, 2])
    d = math.sqrt(sigma_pose[0, 0] + sigma_pose[1, 1])  # cone length
    theta1 = theta - sigma_theta / 2
    theta2 = theta + sigma_theta / 2
    x0 = pose[0]
    y0 = pose[1]
    x1 = x0 + d * math.cos(theta1)
    y1 = y0 + d * math.sin(theta1)
    x2 = x0 + d * math.cos(theta2)
    y2 = y0 + d * math.sin(theta2)
    plt.plot((x0, x1), (y0, y1), 'c-', linewidth=0.5)
    plt.plot((x0, x2), (y0, y2), 'c-', linewidth=0.5)


def main(args=None):
    rclpy.init(args=args)
    plot_odom = PlotOdom()
    while not plot_odom.target_reached:
        rclpy.spin_once(plot_odom)

    rclpy.shutdown()
