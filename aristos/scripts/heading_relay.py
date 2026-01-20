#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from collections import deque
from threading import Lock
import tf
from tf import transformations

class HeadingRelay:
    def __init__(self):
        # Parameters
        self.odom_topic = rospy.get_param('~odom_topic', 'odom')
        self.imu_topic = rospy.get_param('~imu_topic', 'imu')
        self.repub_imu_topic = rospy.get_param('~repub_imu_topic', 'imu_filtered')
        self.repub_odom_topic = rospy.get_param('~repub_odom_topic', 'odom_filtered')
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.0)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 0.5)
        self.window_size = rospy.get_param('~window_size', 10)
        self.timeout = rospy.get_param('~timeout', 10)
        
        # State variables
        self.reset()
        self.tf_listener = tf.TransformListener()

        # Locks
        self.lock = Lock()

        # Subscribers
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        
        # Publisher
        self.imu_pub = rospy.Publisher(self.repub_imu_topic, Imu, queue_size=10)
        self.odom_pub = rospy.Publisher(self.repub_odom_topic, Odometry, queue_size=10)

    def reset(self):
        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.distances = deque(maxlen=self.window_size)
        self.current_velocity = 0.0
        self.odom_msg = Odometry()

    def odom_callback(self, msg):
        with self.lock:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            current_time = msg.header.stamp.to_sec()
            
            if self.prev_x is not None and self.prev_y is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > self.timeout:
                    self.reset()
                elif dt > 0:
                    dx = x - self.prev_x
                    dy = y - self.prev_y
                    distance = math.sqrt(dx**2 + dy**2)
                    velocity = distance / dt
                    
                    self.distances.append(distance)
                    self.current_velocity = velocity

                    # x_dot = dx/dt
                    # y_dot = dy/dt
                    # self.odom_msg.twist.twist.linear.x = x_dot
                    # self.odom_msg.twist.twist.linear.y = y_dot

                    # Compute twist in the child_frame_id reference frame
                    try:
                        (trans, quat) = self.tf_listener.lookupTransform(msg.child_frame_id, msg.header.frame_id, rospy.Time(0))
                        R = transformations.quaternion_matrix(quat)

                        twist_x = (dx / dt) * R[0][0] - (dy / dt) * R[0][1]
                        twist_y = (dx / dt) * R[1][0] + (dy / dt) * R[1][1]
                        self.odom_msg = msg
                        self.odom_msg.twist.twist.linear.x = twist_x
                        self.odom_msg.twist.twist.linear.y = twist_y
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        rospy.logwarn("Transform lookup failed")
                
            self.prev_x = x
            self.prev_y = y
            self.prev_time = current_time
        
    def imu_callback(self, msg):
        with self.lock:
            total_distance = sum(self.distances)
            current_time = msg.header.stamp.to_sec()
            if self.prev_time is not None:
                # print("total_distance: {}".format(total_distance))
                # print("current_velocity: {}".format(self.current_velocity))
                if total_distance > self.distance_threshold and self.current_velocity > self.velocity_threshold and current_time-self.prev_time<self.timeout:
                    self.imu_pub.publish(msg)
                    self.odom_pub.publish(self.odom_msg)
        
if __name__ == '__main__':
    try:
        rospy.init_node('heading_relay', anonymous=True)
        hr = HeadingRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
