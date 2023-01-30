import rclpy
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import Odometry
from rclpy.node import Node
import numpy as np
import math


def eulerFromQuaternion(qw, qx, qy, qz):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2))
    pitch = math.asin(2 * (qw * qy - qz * qx))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2))

    return roll, pitch, yaw


def quaternionFromEuler(r, p, y):
    cr = np.cos(r / 2)
    sr = np.sin(r / 2)
    cp = np.cos(p / 2)
    sp = np.sin(p / 2)
    cy = np.cos(y / 2)
    sy = np.sin(y / 2)

    quat = np.empty((4,))
    quat[0] = cp * sr * cy - sp * cp * sy
    quat[1] = cp * sr * sy + sp * sp * sy
    quat[2] = cp * cr * sy - sp * sp * cy
    quat[3] = cp * cr * cy + sp * sp * sy

    return quat


class ProbabilisticDiffDrive(Node):
    def __init__(self):
        super().__init__("probabilistic_diffdrive_node")

        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_time = None
        self.k_d = 0.02 ** 2
        self.k_theta = (5 * np.pi / 180) ** 2 / (360 * np.pi / 180)
        self.k_drift = (2 * np.pi / 180) ** 2
        self.lastSigma = np.zeros((3, 3))

        self.cov_x_x = 0
        self.cov_x_y = 0
        self.cov_x_theta = 0

        self.cov_y_x = 0
        self.cov_y_y = 0
        self.cov_y_theta = 0

        self.cov_theta_x = 0
        self.cov_theta_y = 0
        self.cov_theta_theta = 0

        self.subscriber = self.create_subscription(Odometry, "/odom", self.callback, 10)
        self.publisher = self.create_publisher(Odometry, "/odom_with_noise", 10)

    def publish(self, pose, time, header_frame_id, child_frame_id):
        msg = Odometry()
        msg.header.stamp = time
        msg.header.frame_id = header_frame_id
        msg.child_frame_id = child_frame_id
        msg.pose = pose

        self.publisher.publish(msg)

    def callback(self, msg):
        current_time = msg.header.stamp
        if self.last_time is None:
            self.last_time = current_time
            return

        delta_time = (current_time.nanosec - self.last_time.nanosec) / 1e+9
        self.last_time = current_time

        # Extract current state from message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        roll, pitch, yaw = eulerFromQuaternion(quat.w, quat.x, quat.y, quat.z)

        # Calculate control input
        linearVelocity = msg.twist.twist.linear.x
        angularVelocity = msg.twist.twist.angular.z

        u = np.array([linearVelocity, angularVelocity])

        o_vk = abs((self.k_d / delta_time) * linearVelocity)  # Speed
        o_wk = abs(((self.k_theta / delta_time) * angularVelocity) + (
                (self.k_drift / delta_time) * linearVelocity))  # Angle + Drift

        S_uk = np.array([[o_vk, 0], [0, o_wk]])

        # Calculate Jacobian
        F_xk = np.array([[1, 0, -delta_time * u[0] * np.sin(yaw)],
                         [0, 1, delta_time * u[0] * np.cos(yaw)],
                         [0, 0, 1]])

        F_uk = np.array([[delta_time * np.cos(yaw), 0],
                         [delta_time * np.sin(yaw), 0],
                         [0, delta_time]])

        S_xk = (F_xk @ self.lastSigma @ F_xk.T) + (F_uk @ S_uk @ F_uk.T)
        self.lastSigma = S_xk

        self.x = x + np.random.normal(self.k_d, self.k_d)
        self.y = y + np.random.normal(self.k_d, self.k_d)
        thetaNorm = np.random.normal(self.k_theta, self.k_theta)
        driftNorm = np.random.normal(self.k_drift, self.k_drift)
        self.theta = yaw + thetaNorm + driftNorm
        quat = quaternionFromEuler(0, 0, self.theta)

        # Create pose message
        pose = PoseWithCovariance()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.covariance[0] = self.lastSigma[0, 0]
        pose.covariance[1] = self.lastSigma[0, 1]
        pose.covariance[5] = self.lastSigma[0, 2]
        pose.covariance[6] = self.lastSigma[1, 0]
        pose.covariance[7] = self.lastSigma[1, 1]
        pose.covariance[11] = self.lastSigma[1, 2]
        pose.covariance[30] = self.lastSigma[2, 0]
        pose.covariance[31] = self.lastSigma[2, 1]
        pose.covariance[35] = self.lastSigma[2, 2]

        # Publish message
        self.publish(pose, current_time, msg.header.frame_id, msg.child_frame_id)


def main(args=None):
    rclpy.init(args=args)
    probabilistic_diffdrive = ProbabilisticDiffDrive()
    rclpy.spin(probabilistic_diffdrive)
    rclpy.shutdown()
