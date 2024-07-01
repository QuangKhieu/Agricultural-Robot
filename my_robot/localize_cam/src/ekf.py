#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import TwistStamped
import message_filters

class EKF:
    def __init__(self, P, Q, R, wheel_distance, wheel_radius) -> None:
        self.P = P
        self.Q = Q
        self.R = R
        self.sampling_time = 0.1
        self.wheel_distance = wheel_distance
        self.wheel_radius = wheel_radius
        self.ekf_pose = np.array([0, 0, 0])
        self.measure_pose = np.array([0, 0, 0])
        self.odometry_pose = np.array([0, 0, 0])
        self.v, self.w = 0, 0

        self.point_pub = rospy.Publisher("no_flag_ekf_pose", Point, queue_size=10)
        self.mes_subscriber = message_filters.Subscriber("measured_pose", PointStamped)
        self.odo_subscriber = message_filters.Subscriber("odometry_velocity", TwistStamped)
        self.ts = message_filters.TimeSynchronizer([self.mes_subscriber, self.odo_subscriber], 10)
        self.ts.registerCallback(self.callback)

    def coefficient_matrices(self):
        pos_est_T = self.odometry_pose[2]

        dS = self.sampling_time * self.v
        dTheta = self.sampling_time * self.w

        self.H = np.eye(3)
        self.V = np.eye(3)

        self.A = np.array([[1, 0, -dS * np.sin(pos_est_T + dTheta/2)],
                    [0, 1,  dS * np.cos(pos_est_T + dTheta/2)],
                    [0, 0,  1]])
        
        self.W = self.sampling_time * self.wheel_radius / 2 * np.array([[np.cos(pos_est_T + dTheta/2) - (dS/self.wheel_distance)*np.sin(pos_est_T + dTheta/2), np.cos(pos_est_T + dTheta/2) + (dS/self.wheel_distance)*np.sin(pos_est_T + dTheta/2)],
                                                    [np.sin(pos_est_T + dTheta/2) + (dS/self.wheel_distance)*np.cos(pos_est_T + dTheta/2), np.sin(pos_est_T + dTheta/2) - (dS/self.wheel_distance)*np.cos(pos_est_T + dTheta/2)],
                                                    [2 / self.wheel_distance, 2 / self.wheel_distance]])

    def process_ekf(self):
        # 1. Prediction step
        # priori estimation state
        pri_est_X = self.odometry_pose

        # priori error covariance
        self.coefficient_matrices()
        self.P = self.A @ self.P @ self.A.T + self.W @ self.Q @ self.W.T

        # 2. Update (Correction) step
        # Kalman gain matrix
        self.K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.V @ self.R @ self.V.T)
        # posteriori estimation state
        mes_Z = self.measure_pose
        pos_est_X = pri_est_X + self.K @ (mes_Z - pri_est_X)
        self.ekf_pose = pos_est_X

        # error covariance update
        self.P = (np.eye(3) - self.K @ self.H) @ self.P

        # print to check
        print()
        print(1, self.ekf_pose)
        print(2, self.measure_pose)
        print(3, self.odometry_pose)
        print()

    def callback(self, mes_msg, odo_msg):
        mes_pose = mes_msg.point
        odo_pose = odo_msg.twist
        self.measure_pose = np.array([mes_pose.x, mes_pose.y, mes_pose.z])
        self.odometry_pose = np.array([odo_pose.linear.x, odo_pose.linear.y, odo_pose.linear.z])
        self.v = odo_pose.angular.x
        self.w = odo_pose.angular.y
        self.sampling_time = odo_pose.angular.z

        self.process_ekf()

        ekf_point = Point(self.ekf_pose[0], self.ekf_pose[1], self.ekf_pose[2])
        self.point_pub.publish(ekf_point)


if __name__ == "__main__":
    # localize_with_barcode_and_sensors()
    # use service to obtain odometry estimation, landmark measurement, and velocity
    rate = 2.0    # Hz
    sampling_time = 1 / rate   # in s, its a time step between 2 estimations
    wheel_distance = 0.30 # m
    wheel_radius = 0.03   # m

    # initialize error covariance matrices, will have to update over time
    P = np.eye(3) 
    Q = np.array([[0.12, 0],
                  [0, 0.12]])
    R = np.array([[0.00228,   0,   0],
                  [  0, 0.001421,   0],
                  [  0,   0, 0.00009]])

    try:
        rospy.init_node("ekf", anonymous=True)
        EKF_process = EKF(P, Q, R, wheel_distance, wheel_radius)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown!")
