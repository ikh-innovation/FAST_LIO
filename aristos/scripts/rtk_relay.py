#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

class RTKRelay:
    def __init__(self):
        # Initialize node
        rospy.init_node('rtk_relay_node')

        # Initialize service
        self.relay_service = rospy.Service("use_rtk", SetBool, self.use_rtk_cb)

        # Initialize subscribers
        self.rtk_sub = rospy.Subscriber("odometry/gps/onlyRTK", Odometry, self.rtk_callback)

        # Initialize publishers
        self.rtk_pub = rospy.Publisher("relay/odometry/gps/onlyRTK", Odometry, queue_size=10)

        # Default to lidar source
        self.use_rtk = True

        # Storage for latest messages
        self.latest_rtk_msg = None

    def rtk_callback(self, msg):
        self.latest_rtk_msg = msg
        if self.use_rtk:
            self.rtk_pub.publish(self.latest_rtk_msg)

    def use_rtk_cb(self, req):
        self.use_rtk = req.data
        response = SetBoolResponse()
        response.success = True
        response.message = "Start using RTK" if self.use_rtk else "Stop using RTK"
        return response

if __name__ == "__main__":
    try:
        relay_node = RTKRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
