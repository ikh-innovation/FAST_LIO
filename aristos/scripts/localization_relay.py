#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

class LocalizationRelay:
    def __init__(self):
        # Initialize node
        rospy.init_node('localization_relay_node')

        # Initialize service client
        rospy.wait_for_service('halt_lio', timeout=20)
        self.halt_lio = rospy.ServiceProxy('halt_lio', SetBool)

        # Initialize service
        self.switch_service = rospy.Service("use_lidar_odom", SetBool, self.use_lidar_odom)

        # Initialize subscribers
        self.imu_sub = rospy.Subscriber("imu", Imu, self.imu_callback)
        self.wheel_odom_sub = rospy.Subscriber("wheel_odom", Odometry, self.wheel_odom_callback)
        # self.lidar_odom_sub = rospy.Subscriber("lidar_odom", Odometry, self.lidar_odom_callback)

        # Initialize publishers
        self.imu_pub = rospy.Publisher("relay/imu", Imu, queue_size=10)
        self.wheel_odom_pub = rospy.Publisher("relay/wheel_odom", Odometry, queue_size=10)
        # self.lidar_odom_pub = rospy.Publisher("relay/lidar_odom", Odometry, queue_size=10)

        # Default to lidar source
        self.use_lidar = True

        # Storage for latest messages
        self.latest_imu_msg = None
        self.latest_wheel_odom_msg = None
        self.latest_lidar_odom_msg = None

    def imu_callback(self, msg):
        self.latest_imu_msg = msg
        if not self.use_lidar:
            self.imu_pub.publish(self.latest_imu_msg)

    def wheel_odom_callback(self, msg):
        self.latest_wheel_odom_msg = msg
        if not self.use_lidar:
            self.wheel_odom_pub.publish(self.latest_wheel_odom_msg)

    def lidar_odom_callback(self, msg):
        self.latest_lidar_odom_msg = msg
        if self.use_lidar:
            self.lidar_odom_pub.publish(self.latest_lidar_odom_msg)

    def use_lidar_odom(self, req):
        self.use_lidar = req.data
        try:
            self.halt_lio(not self.use_lidar)
        except rospy.ServiceException as e:
            estr = "halt LIO :Service call failed: {}".format(str(e))
            rospy.logerr(estr)
            response = SetBoolResponse()
            response.success = False
            response.message = estr
        response = SetBoolResponse()
        response.success = True
        response.message = "Using lidar odometry" if self.use_lidar else "Using wheel/IMU odometry"
        return response

if __name__ == "__main__":
    try:
        relay_node = LocalizationRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
