#!/usr/bin/env python

import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class Republisher:
    def __init__(self):
        rospy.init_node("republisher", anonymous=True)

        # Load parameters
        self.rtk_odom_params = rospy.get_param("~rtk_odom", {})
        self.wheel_odom_params = rospy.get_param("~wheel_odom", {})
        self.imu_params = rospy.get_param("~imu", {})

        # Publishers
        self.rtk_odom_pub = rospy.Publisher("rtk_odom/repub", Odometry, queue_size=10)
        self.wheel_odom_pub = rospy.Publisher("wheel_odom/repub", Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher("imu/repub", Imu, queue_size=10)

        # Subscribers
        rospy.Subscriber("rtk_odom", Odometry, self.rtk_odom_callback)
        rospy.Subscriber("wheel_odom", Odometry, self.wheel_odom_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)

    def modify_covariance(self, cov_list, default_cov):
        return cov_list if cov_list else default_cov

    def rtk_odom_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = self.rtk_odom_params.get("frame_id", msg.header.frame_id)
        modified_msg.child_frame_id = self.rtk_odom_params.get("child_frame_id", msg.child_frame_id)
        modified_msg.pose.covariance = self.modify_covariance(self.rtk_odom_params.get("pose_covariance"), msg.pose.covariance)
        modified_msg.twist.covariance = self.modify_covariance(self.rtk_odom_params.get("twist_covariance"), msg.twist.covariance)
        self.rtk_odom_pub.publish(modified_msg)

    def wheel_odom_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = self.wheel_odom_params.get("frame_id", msg.header.frame_id)
        modified_msg.child_frame_id = self.wheel_odom_params.get("child_frame_id", msg.child_frame_id)
        modified_msg.pose.covariance = self.modify_covariance(self.wheel_odom_params.get("pose_covariance"), msg.pose.covariance)
        modified_msg.twist.covariance = self.modify_covariance(self.wheel_odom_params.get("twist_covariance"), msg.twist.covariance)
        self.wheel_odom_pub.publish(modified_msg)

    def imu_callback(self, msg):
        modified_msg = msg
        modified_msg.header.frame_id = self.imu_params.get("frame_id", msg.header.frame_id)
        modified_msg.orientation_covariance = self.modify_covariance(self.imu_params.get("orientation_covariance"), msg.orientation_covariance)
        modified_msg.angular_velocity_covariance = self.modify_covariance(self.imu_params.get("angular_velocity_covariance"), msg.angular_velocity_covariance)
        modified_msg.linear_acceleration_covariance = self.modify_covariance(self.imu_params.get("linear_acceleration_covariance"), msg.linear_acceleration_covariance)
        self.imu_pub.publish(modified_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = Republisher()
    node.run()
