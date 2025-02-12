#!/usr/bin/env python

import math
import sys
import rospy

from cob_phidgets.srv import SetDigitalSensor, SetDigitalSensorRequest, SetDigitalSensorResponse
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerResponse

class SpatialControlActionsSyncOutput:

    def __init__(self):
        rospy.init_node("sca_sync_output")
        
        self.output_set = False
        
        # TODO: Make this resetable over service oder dyn-reconfigure 
        self.dead_zone_level = 0.01

        # initialize publishers and action clients
        # TODO: Make this resetable over service oder dyn-reconfigure 
        self.sca_output_sub = rospy.Subscriber("input_data", Vector3, self.sca_output_callback)
        
        self.sync_signal_debug_pub = rospy.Publisher("sync_signal_debug", Int8, queue_size=1)
        
        self.phigets_set_service = rospy.ServiceProxy("set_service", SetDigitalSensor)
        self.request = SetDigitalSensorRequest()
        self.request.uri = "sync_signal"
        
        self.set_service = rospy.Service("manual_trigger", Trigger, self.send_sync_signal)


    def sca_output_callback(self, data):
        value = math.sqrt(data.x*data.x + data.y*data.y)
        if (not self.output_set):
            if value > self.dead_zone_level:
                self.request.state = 1;
                resp = self.phigets_set_service(self.request)
                self.sync_signal_debug_pub.publish(self.request.state)
                self.output_set = True
                rospy.loginfo("Phidgets output _ON_!")
                
        else:
            if value <= self.dead_zone_level:
                self.request.state = 0
                resp = self.phigets_set_service(self.request)
                self.sync_signal_debug_pub.publish(self.request.state)
                self.output_set = False
                rospy.loginfo("Phidges output _OFF_!")


    def send_sync_signal(self, req):
        res = TriggerResponse()
        res.success = True
      
        self.request.state = 1;
        self.phigets_set_service(self.request)
        self.sync_signal_debug_pub.publish(self.request.state)
        rospy.sleep(rospy.Duration(0.2))
        self.request.state = 0;
        resp = self.phigets_set_service(self.request)
        self.sync_signal_debug_pub.publish(self.request.state)
        
        return res


def main(args):
    sca_sync = SpatialControlActionsSyncOutput()
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)

