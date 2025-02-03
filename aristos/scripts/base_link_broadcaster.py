#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from nav_msgs.msg import Odometry

class OdomTransformer:
    def __init__(self):
        rospy.init_node("base_link_broadcaster", anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf.TransformBroadcaster()

        self.odom_sub = rospy.Subscriber("Odometry", Odometry, self.odom_callback)
        # self.odom_pub = rospy.Publisher("/aristos_lvx/Odometry_transformed", Odometry, queue_size=10)
    
    def odom_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform("aristos_lvx/velodyne", "aristos_lvx/base_link", rospy.Time(0), rospy.Duration(1.0))

 
            self.br.sendTransform((transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
                            (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
                            rospy.Time.now(),
                            "aristos_lvx/base_linkTF",  # Target frame (base frame of the robot)
                            "aristos_lvx/velodyne_lio")  # Source frame (frame in which odometry is expressed)

        except tf2_ros.LookupException as e:
            rospy.logwarn("Transform lookup failed: {}".format(e))

if __name__ == "__main__":
    OdomTransformer()
    rospy.spin()
