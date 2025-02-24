#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from tf import transformations
import message_filters
from collections import deque
import numpy as np
from tf import transformations
from threading import Lock

class ImuModifier:
    def __init__(self):
        rospy.init_node("imu_modifier", anonymous=True)

        # Params
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)
        self.variance = rospy.get_param('~variance', 0.01)
        
        self.pub = rospy.Publisher("imu/data/modified", Imu, queue_size=10)
        self.sub = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.service = rospy.Service("imu/modified/halt_publishing", SetBool, self.handle_halt_request)
        
        self.halt_publishing = False

    def imu_callback(self, msg):
        if self.halt_publishing:
            return
        
        modified_msg = msg
        q_offset = transformations.quaternion_from_euler(0, 0, self.yaw_offset)
        q_modified = transformations.quaternion_multiply([modified_msg.orientation.x, modified_msg.orientation.y, modified_msg.orientation.z, modified_msg.orientation.w], q_offset)
        modified_msg.orientation.x = q_modified[0]
        modified_msg.orientation.y = q_modified[1]
        modified_msg.orientation.z = q_modified[2]
        modified_msg.orientation.w = q_modified[3]

        modified_msg.orientation_covariance = [
            self.variance, 0, 0,
            0, self.variance, 0,
            0, 0, self.variance]
        
        self.pub.publish(modified_msg)
    
    def handle_halt_request(self, req):
        self.halt_publishing = req.data
        return SetBoolResponse(success=True, message="Publishing halted" if req.data else "Publishing resumed")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ImuModifier()
    node.run()

