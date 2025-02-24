#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from threading import Lock

class OdometryIMURelayNode:
    def __init__(self):
        rospy.init_node('imu_onlyRTK_relay_node')

        # Parameters
        self.inactivity_timeout = rospy.get_param('~inactivity_timeout', 1.0) 
        self.validity_timeout = rospy.get_param('~validity_timeout', 5.0) 

        # Inits
        self.last_odom_time = rospy.Time.now()
        self.first_odom_time = []
        self.imu_data = None
        self.lock = Lock()

        # Subscribers
        self.odom_sub = rospy.Subscriber('onlyRTK', Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber('imu/data', Imu, self.imu_callback)

        # Publisher
        self.imu_relay_pub = rospy.Publisher('imu_onlyRTK/relay', Imu, queue_size=10)

    def odom_callback(self, msg):
        """ Update last time odometry was received """
        with self.lock:
            self.last_odom_time = rospy.Time.now()
            if not self.first_odom_time:
                self.first_odom_time = self.last_odom_time

    def imu_callback(self, msg):
        self.imu_data = msg
        with self.lock:
            # self.imu_data.orientation_covariance = [
            # 0.01, 0, 0,
            # 0, 0.01, 0,
            # 0, 0, 0.01
            # ]
            if (rospy.Time.now() - self.last_odom_time).to_sec() < self.inactivity_timeout:
                if self.first_odom_time and (rospy.Time.now() - self.first_odom_time).to_sec() > self.validity_timeout:
                    self.imu_relay_pub.publish(self.imu_data)
            else:
                self.first_odom_time = []

if __name__ == '__main__':
    try:
        node = OdometryIMURelayNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
