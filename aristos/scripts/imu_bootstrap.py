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

class ImuBootstrap:
    def __init__(self):
        rospy.init_node("imu_bootstrap", anonymous=True)

        # Params
        self.yaw_offset = rospy.get_param('~yaw_offset', 0.0)
        self.variance = rospy.get_param('~variance', 0.01)
        self.speed_ths = rospy.get_param('~speed_ths', 0.15)
        self.speed_buffer = rospy.get_param('~speed_buffer', 25)

        # init
        self.imu_msg = Imu() #rospy.wait_for_message("imu/data", Imu, timeout=30)
        self.message_time_prv = rospy.Time.now()
        self.error_angle_prv = []
        self.yaw = 0

        # Control
        # 0.005, 0.5, 2
        self.Kp = 0.005 
        self.Kd = 0.0
        self.buffer_secs = 2 # secs

        # self.active_ekf_sub = rospy.Subscriber("active_ekf/odom", Odometry, self.active_callback)
        # self.bootstrap_ekf_sub = rospy.Subscriber("bootstrap_ekf/odom", Odometry, self.bootstrap_callback)

        # Time synchronizer
        self.active_ekf_sub = message_filters.Subscriber("active_ekf/odom", Odometry)
        self.bootstrap_ekf_sub = message_filters.Subscriber("bootstrap_ekf/odom", Odometry)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.active_ekf_sub, self.bootstrap_ekf_sub], 10, slop=2*1.0/50)
        self.ts.registerCallback(self.odom_callback)

        #
        self.odom_callback_hz = 50 # ekf runs at 50hz, but due to time sync we actually have 40
        self.MAXLEN = self.odom_callback_hz*self.buffer_secs 
        self.odom_deque = deque(maxlen=self.MAXLEN)
        self.twist_deque = deque(maxlen=self.MAXLEN)

        #
        self.imu_lock = Lock()
        self.start_control_lock = Lock()
        
        rospy.wait_for_service('use_rtk', timeout=20)
        self.use_rtk = rospy.ServiceProxy('use_rtk', SetBool)


        self.pub = rospy.Publisher("imu/data/modified", Imu, queue_size=10)
        self.sub = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.service = rospy.Service("imu/modified/halt_publishing", SetBool, self.handle_halt_request)
        
        self.start_control = False

    # # ====== control yaw ======= 
    # def odom_callback(self, active_odom, bootstrap_odom):        
    #     self.odom_deque.append([active_odom.pose.pose.position, bootstrap_odom.pose.pose.position])
    #     self.corrected_imu_msg = self.imu_msg

    #     if len(self.odom_deque)>=self.MAXLEN and self.start_control:
    #         error_angle = self.angle_between_two_vectors(self.odom_deque[0][0], self.odom_deque[-1][0], self.odom_deque[0][1], self.odom_deque[-1][1])
    #         self.yaw = self.yaw - 0.01*error_angle
    #         q_corrected = transformations.quaternion_from_euler(0, 0, self.yaw)
    #         self.corrected_imu_msg.orientation.x = q_corrected[0]
    #         self.corrected_imu_msg.orientation.y = q_corrected[1]
    #         self.corrected_imu_msg.orientation.z = q_corrected[2]
    #         self.corrected_imu_msg.orientation.w = q_corrected[3]
    #     else:
    #         angles = transformations.euler_from_quaternion([self.corrected_imu_msg.orientation.x, self.corrected_imu_msg.orientation.y, self.corrected_imu_msg.orientation.z, self.corrected_imu_msg.orientation.w])
    #         self.yaw = angles[2]

    #     self.pub.publish(self.corrected_imu_msg)

    # ====== control yaw_offset ======= 
    # def odom_callback(self, active_odom, bootstrap_odom):   
    #     self.odom_deque.append([active_odom.pose.pose.position, bootstrap_odom.pose.pose.position])
    #     with self.imu_lock:
    #         self.corrected_imu_msg = self.imu_msg

    #     with self.start_control_lock:
    #         start_control = self.start_control

    #     self.message_time = active_odom.header.stamp.to_sec()

    #     if self.message_time_prv != self.message_time:
    #         if len(self.odom_deque)>=self.MAXLEN and start_control:
    #             error_angle = self.angle_between_two_vectors(self.odom_deque[0][1], self.odom_deque[-1][1], self.odom_deque[0][0], self.odom_deque[-1][0])
    #             if self.error_angle_prv:
    #                 error_angle_dot = (error_angle - self.error_angle_prv)/(self.message_time - self.message_time_prv)
    #             else:
    #                 error_angle_dot = 0

    #             self.yaw = self.yaw + self.Kp*error_angle + self.Kd*error_angle_dot
    #             q_offset = transformations.quaternion_from_euler(0, 0, self.yaw)
    #             q_corrected = transformations.quaternion_multiply([self.corrected_imu_msg.orientation.x, self.corrected_imu_msg.orientation.y, self.corrected_imu_msg.orientation.z, self.corrected_imu_msg.orientation.w], q_offset)
    #             self.corrected_imu_msg.orientation.x = q_corrected[0]
    #             self.corrected_imu_msg.orientation.y = q_corrected[1]
    #             self.corrected_imu_msg.orientation.z = q_corrected[2]
    #             self.corrected_imu_msg.orientation.w = q_corrected[3]

    #             self.error_angle_prv = error_angle
                
    #         else:
    #             self.message_time_prv = self.message_time
    #             self.yaw = 0.0 #self.yaw_offset

    #         self.pub.publish(self.corrected_imu_msg)
    #     else:
    #         self.message_time_prv = self.message_time
        
    # ====== control yaw_offset 2 ======= 

    def odom_callback(self, active_odom, bootstrap_odom):   
        self.odom_deque.append([np.array([active_odom.pose.pose.position.x, active_odom.pose.pose.position.y]), 
                                np.array([bootstrap_odom.pose.pose.position.x, bootstrap_odom.pose.pose.position.y])])
        
        self.twist_deque.append(np.array([active_odom.twist.twist.linear.x, active_odom.twist.twist.linear.y]))

        with self.imu_lock:
            self.corrected_imu_msg = self.imu_msg

        with self.start_control_lock:
            start_control = self.start_control

        self.message_time = rospy.Time.now().to_sec() #active_odom.header.stamp.to_sec()

        if len(self.odom_deque)>=self.MAXLEN and start_control:
            print("======================== STARTED ==================================================")
            if self.is_chunk_valid(self.odom_deque, self.twist_deque, twist_length=self.speed_buffer):
                print("======================== VALID ==================================================")
                print(self.message_time - self.message_time_prv)
                error_angle = self.angle_between_two_np_vectors(self.odom_deque[0][1], self.odom_deque[-1][1], self.odom_deque[0][0], self.odom_deque[-1][0])
                if self.error_angle_prv and self.message_time - self.message_time_prv>0:
                    error_angle_dot = (error_angle - self.error_angle_prv)/(self.message_time - self.message_time_prv)
                else:
                    error_angle_dot = 0
                self.yaw = self.yaw + self.Kp*error_angle + self.Kd*error_angle_dot

                q_offset = transformations.quaternion_from_euler(0, 0, self.yaw)
                q_corrected = transformations.quaternion_multiply([self.corrected_imu_msg.orientation.x, self.corrected_imu_msg.orientation.y, self.corrected_imu_msg.orientation.z, self.corrected_imu_msg.orientation.w], q_offset)
                self.corrected_imu_msg.orientation.x = q_corrected[0]
                self.corrected_imu_msg.orientation.y = q_corrected[1]
                self.corrected_imu_msg.orientation.z = q_corrected[2]
                self.corrected_imu_msg.orientation.w = q_corrected[3]

                self.error_angle_prv = error_angle

                self.pub.publish(self.corrected_imu_msg)

            else:
                # reseting
                print("======================== RESETTING ==================================================")
                self.error_angle_prv = []
                self.yaw = 0

        else:
           print("======================== STOPPED ==================================================")
           self.pub.publish(self.corrected_imu_msg) 

        self.message_time_prv = self.message_time
        

    def is_chunk_valid(self, poses, twists, twist_length=1):
        if len(poses)<=2:
            raise Exception("Too few points")
        
        dists = self.total_distances(poses)
        mean_speed = self.mean_speed(twists, twist_length)

        print("//////// Distances: {}, Mean speed: {}".format(dists, mean_speed))

        # TODO: refine the following
        # if dists[0]/dists[1]>1.2 or dists[0]/dists[1]<1.0/1.2:
        #     raise Exception("Paths vary significantly")
        
        # TODO: surface vs dist checking can be added here

        # distance travelled vs distance computed with minimum acceptable speed
        if dists[0]>self.buffer_secs*self.speed_ths and mean_speed>self.speed_ths:
            return True
        else:
            return False
        
    def total_distances(self, d):
        arr = np.array(d)
        diffs = np.diff(arr, axis=0)
        distances = np.linalg.norm(diffs, axis=2) 
        return np.sum(distances, axis=0)
    
    def mean_speed(self, t, lenght = 1):
        if lenght>len(t):
            rospy.logwarn("buffer lenght exceed deque length. Setting to deque length...")
            n = len(t)
        else:
            n = lenght
        tnp = np.array(t)
        return np.linalg.norm(np.mean(tnp[-n:], axis=0))
        
    def angle_between_two_np_vectors(self, v1_start, v1_end, v2_start, v2_end):
        v1 = np.array([v1_end[0] - v1_start[0], v1_end[1] - v1_start[1]])
        v2 = np.array([v2_end[0] - v2_start[0], v2_end[1] - v2_start[1]])
        return np.math.atan2(np.linalg.det([v1,v2]),np.dot(v1,v2))

    def angle_between_two_vectors(self, v1_start, v1_end, v2_start, v2_end):
        v1 = np.array([v1_end.x - v1_start.x, v1_end.y - v1_start.y])
        v2 = np.array([v2_end.x - v2_start.x, v2_end.y - v2_start.y])
        return np.math.atan2(np.linalg.det([v1,v2]),np.dot(v1,v2))
    
    def imu_callback(self, msg):
        with self.imu_lock:
            self.imu_msg = msg
    
    def handle_halt_request(self, req):
        with self.start_control_lock:
            self.start_control = req.data
        # self.use_rtk(not req.data)
        return SetBoolResponse(success=True, message="Publishing halted" if req.data else "Publishing resumed")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ImuBootstrap()
    node.run()

