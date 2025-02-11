#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
import rosservice

class ServiceAggregator:
    def __init__(self, service_name):
        self.service_name = service_name  # The service name to proxy (e.g., 'awesome_service')
        self.proxies = []
        
        rospy.Timer(rospy.Duration(5), self.update_service_proxies)  # Update service list every 5s
        self.server = rospy.Service("aggregated_" + service_name, SetBool, self.handle_request) 
        rospy.loginfo("Service aggregator node started")

    def update_service_proxies(self, event):
        """Finds all services matching the name under any namespace and creates proxies"""
        self.proxies = []
        all_services = rosservice.get_service_list()
        matched_services = [s for s in all_services if s.endswith('/' + self.service_name)]
        
        for srv in matched_services:
            try:
                rospy.wait_for_service(srv, timeout=2)  # Ensure the service is available
                proxy = rospy.ServiceProxy(srv, SetBool)
                self.proxies.append(proxy)
                rospy.loginfo("Added proxy for: {}".format(srv))
            except rospy.ROSException:
                rospy.logwarn("Service {} not available".format(srv))
        
    def handle_request(self, req):
        """Forwards the request to all proxies and aggregates responses"""
        responses = []
        success = True
        message = ""
        
        for proxy in self.proxies:
            try:
                res = proxy(SetBoolRequest())  
                responses.append(res.message)
                if not res.success:
                    success = False
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: {}".format(e))
                success = False
        
        message = "; ".join(responses)
        return SetBoolResponse(success=success, message=message)

if __name__ == "__main__":
    rospy.init_node("service_aggregator")
    service_name = rospy.get_param("~service_name", "use_rtk")
    aggregator = ServiceAggregator(service_name)
    rospy.spin()