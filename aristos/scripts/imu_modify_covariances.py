#!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Imu

# def imu_callback(msg):
#     # Modify covariance values
#     modified_msg = msg
#     modified_msg.orientation_covariance = [
#         0.1, 0, 0,
#         0, 0.1, 0,
#         0, 0, 0.1
#     ]
#     # modified_msg.angular_velocity_covariance = [
#     #     0.02, 0, 0,
#     #     0, 0.02, 0,
#     #     0, 0, 0.02
#     # ]
#     # modified_msg.linear_acceleration_covariance = [
#     #     0.03, 0, 0,
#     #     0, 0.03, 0,
#     #     0, 0, 0.03
#     # ]
    
#     # Publish the modified message
#     pub.publish(modified_msg)

# if __name__ == "__main__":
#     rospy.init_node("imu_covariance_modifier", anonymous=True)

#     # Define publisher
#     pub = rospy.Publisher("imu/data/modified_covariances", Imu, queue_size=10)
    
#     # Define subscriber
#     rospy.Subscriber("imu/data", Imu, imu_callback)
    
#     rospy.spin()


import rospy
from sensor_msgs.msg import Imu
from std_srvs.srv import SetBool, SetBoolResponse

class ImuCovarianceModifier:
    def __init__(self):
        rospy.init_node("imu_covariance_modifier", anonymous=True)
        
        self.pub = rospy.Publisher("imu/data/modified_covariances", Imu, queue_size=10)
        self.sub = rospy.Subscriber("imu/data", Imu, self.imu_callback)
        self.service = rospy.Service("imu/halt_publishing", SetBool, self.handle_halt_request)
        
        self.halt_publishing = False

    def imu_callback(self, msg):
        if self.halt_publishing:
            return

        modified_msg = msg
        modified_msg.orientation_covariance = [
            0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01
        ]
        # modified_msg.angular_velocity_covariance = [
        #     0.02, 0, 0,
        #     0, 0.02, 0,
        #     0, 0, 0.02
        # ]
        # modified_msg.linear_acceleration_covariance = [
        #     0.03, 0, 0,
        #     0, 0.03, 0,
        #     0, 0, 0.03
        # ]
        
        self.pub.publish(modified_msg)
    
    def handle_halt_request(self, req):
        self.halt_publishing = req.data
        return SetBoolResponse(success=True, message="Publishing halted" if req.data else "Publishing resumed")
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = ImuCovarianceModifier()
    node.run()