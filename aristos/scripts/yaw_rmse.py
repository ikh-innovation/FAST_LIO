#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from collections import deque
from tf.transformations import euler_from_quaternion
from threading import Lock
from math import pi

class YawRMSECalculator:
    def __init__(self):
        rospy.init_node('yaw_rmse_calculator', anonymous=True)
        
        # Parameters
        self.imu_topic = rospy.get_param('~imu_topic', 'imu/data')
        self.odom_topic = rospy.get_param('~odom_topic', 'gnss_heading')
        self.window_size = rospy.get_param('~window_size', 5.0)  # Rolling window in seconds
        self.odometry_update_timeout = rospy.get_param('~odometry_update_timeout', 1.0)  # Timeout due to odom topic inactivity
        self.timer_hz = rospy.get_param('~timer_hz', 10)
        
        # Buffers to store yaw values with timestamps
        self.imu_data = deque()
        self.odom_data = deque()

        self.last_odom_time = None
        self.latest_imu_msg = None
        self.imu_relay_pub = rospy.Publisher('imu_rmse_relay', Imu, queue_size=10)
        self.RMSE_ths = 0.01

        # Locks
        self.odom_lock = Lock()
        self.imu_lock = Lock()
        
        # Subscribers
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        # Timer for RMSE computation
        self.rmse_msg = Float32()
        self.rmse_pub = rospy.Publisher('yaw_rmse', Float32, queue_size=10)
        self.error_msg = Float32()
        self.error_pub = rospy.Publisher('latest_imu_gnssHeading_error', Float32, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/self.timer_hz), self.compute_rmse)

    def imu_callback(self, msg):
        with self.imu_lock:
            self.latest_imu_msg = msg
            yaw = self.get_yaw_from_quaternion(msg.orientation)
            self.imu_data.append((msg.header.stamp.to_sec(), yaw))
            # self.clean_old_imu_data()

    def odom_callback(self, msg):
        with self.odom_lock:
            yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
            self.odom_data.append((msg.header.stamp.to_sec(), yaw))
            # self.clean_old_odom_data()

    def get_yaw_from_quaternion(self, orientation):
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw
    
    def clean_old_imu_data(self):
        current_time = rospy.get_time()
        while self.imu_data and self.imu_data[0][0] < current_time - self.window_size:
            self.imu_data.popleft()

    def clean_old_odom_data(self):
        current_time = rospy.get_time()
        # print("Odom data before")
        # print(self.odom_data)
        # print("Current time:")
        # print(current_time)
        # print("First data:")
        # if self.odom_data:
        #     print(self.odom_data[0][0])
        # else:
        #     print("Empty list")
        while self.odom_data and self.odom_data[0][0] < current_time - self.window_size:
            self.odom_data.popleft()
        # print("Odom data after")
        # print(self.odom_data)
    
    def compute_rmse(self, event):
        with self.imu_lock:
            with self.odom_lock:
                self.relay_imu = False

                self.clean_old_imu_data()
                self.clean_old_odom_data()
                # print("===================== IMU ====================")
                # print(self.imu_data)
                # print("===================== ODOM ====================")
                # print(self.odom_data)
        
                if not self.imu_data or not self.odom_data:
                    return
                
                # return if odom topic is inactive 
                current_time = rospy.get_time()
                if self.last_odom_time:
                    if current_time - self.last_odom_time > self.odometry_update_timeout:
                        return

                imu_times, imu_yaws = zip(*self.imu_data)
                odom_times, odom_yaws = zip(*self.odom_data)
                
                matched_pairs = []

                for odom_t, odom_y in zip(odom_times, odom_yaws):
                    closest_idx = np.argmin(np.abs(np.array(imu_times) - odom_t))
                    # if (np.abs(imu_times[closest_idx] - odom_t)<ths) #TODO: Add threshold
                    matched_pairs.append((odom_y, imu_yaws[closest_idx]))
                if matched_pairs:
                    errors = [self.get_signed_angle_error(odom_y, imu_y) for odom_y, imu_y in matched_pairs]
                    rmse = np.sqrt(np.mean(np.square(errors)))
                    # rospy.loginfo('Yaw RMSE: {}'.format(rmse))

                    self.error_msg.data = errors[-1]
                    self.error_pub.publish(self.error_msg)

                    # print("-----------RMSE THS CHECKING")
                    if rmse<self.RMSE_ths:
                        # self.relay_imu = True
                        # print("-----------RMSE below threshold")
                        if current_time-self.latest_imu_msg.header.stamp.to_sec()<1.0/self.timer_hz:
                            # print("-----------IMU relayed")
                            self.imu_relay_pub.publish(self.latest_imu_msg)
                        
                    self.rmse_msg.data = rmse
                    self.rmse_pub.publish(self.rmse_msg)
    
    def get_signed_angle_error(self, a1, a2):
        angle = a1 - a2
        if angle > pi:
            angle -= 2*pi
        elif angle < -pi:
            angle += 2*pi
        return angle

if __name__ == '__main__':
    try:
        YawRMSECalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
