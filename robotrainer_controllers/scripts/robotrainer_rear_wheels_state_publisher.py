#!/usr/bin/env python

import sys
import rospy
import math

from robotrainer_controllers.srv import SetRearWheelsState, SetRearWheelsStateResponse, GetRearWheelsState, GetRearWheelsStateResponse
from std_srvs.srv import Trigger

from sensor_msgs.msg import JointState

STEER_WHEEL_NEUTRAL_POSITION = {"f_caster_rotation_joint": [0.0, 0],
                                "bl_caster_rotation_joint": [-180, +1],
                                "br_caster_rotation_joint": [0.0, -1],
                               }


STATES_ANGULAR = {1: [16, "CLOSED", 0.0],
                  2: [21, "MIDDLE_CLOSED", 5.0],
                  3: [26, "MIDDLE", 10.0],
                  4: [31, "MIDDLE_OPENED", 15.0],
                  5: [36, "OPENED", 20.0],
                 }

STATES_LINEAR = {1: [0.274, "CLOSED"],
                 2: [0.324, "MIDDLE_CLOSED"],
                 3: [0.374, "MIDDLE_OPENED"],
                 4: [0.424, "OPENED"],
                }

class RoboTrainerRearConfigurationStatePublisher():

    def __init__(self):
        rospy.init_node("robotrainer_rear_wheels_configuration_state_publisher")

        self.frequency = rospy.get_param("~/frequency", 2)
        self.joint_names_angular = rospy.get_param("~/joint_names_angular", ['base_bl_chassis_bl_telescope_rot_joint', 'base_br_chassis_br_telescope_rot_joint'])
        self.joint_names_linear = rospy.get_param("~/joint_names_linear", ['base_bl_chassis_bl_telescope_lin_joint', 'base_br_chassis_br_telescope_lin_joint'])
        self.wheel_controller_prefix = rospy.get_param("~/wheel_controller_prefix", '/base/wheel_controller/wheels')

        self.current_state = {"angular": 1,
                              "linear": 1
                              }

        # Default values for the message
        self.joint_state_message = JointState()
        self.joint_state_message.header.seq = 0;
        self.joint_state_message.name = [self.joint_names_angular[0],
                                         self.joint_names_angular[1],
                                         self.joint_names_linear[0],
                                         self.joint_names_linear[1]
                                        ]

        self.joint_state_publisher = rospy.Publisher("joint_states", JointState, queue_size=1)

        self.set_service = rospy.Service("set_state", SetRearWheelsState, self.set_rear_wheel_state)
        self.get_service = rospy.Service("get_state", GetRearWheelsState, self.get_rear_wheel_state)

        self.update_controller_srv = rospy.ServiceProxy("/base/update_kinematics", Trigger);
        self.update_odometry_controller_srv = rospy.ServiceProxy("/base/odometry_controller/update_kinematics", Trigger);
        self.update_twist_controller_srv = rospy.ServiceProxy("/base/twist_controller/update_kinematics", Trigger);

        self.read_set_default_wheel_parameters();

        while not rospy.is_shutdown():
            self.joint_state_message.header.stamp = rospy.Time.now()
            self.joint_state_message.position = [math.radians(180 - STATES_ANGULAR[self.current_state["angular"]][0]),
                                                 math.radians(180 + STATES_ANGULAR[self.current_state["angular"]][0]),
                                                 STATES_LINEAR[self.current_state["linear"]][0],
                                                 STATES_LINEAR[self.current_state["linear"]][0]
                                                ]
            self.joint_state_publisher.publish(self.joint_state_message)

            rospy.sleep(1.0/self.frequency)


    def set_rear_wheel_state(self, req):

        res = SetRearWheelsStateResponse()
        res.success = True

        if not req.angular_key in STATES_ANGULAR.keys():
            res.success = False
            res.message = "Key 'angular': " + str(req.angular_key) + " does not exists! Keeping the same configuration.\n"
            rospy.logerr(res.message)
        if not req.linear_key in STATES_LINEAR.keys():
            res.success = False
            res.message += "Key 'linear': " + str(req.linear_key) + " does not exists! Keeping the same configuration.\n"
            rospy.logerr(res.message)

        if res.success:
            old_state = self.current_state.copy()
            self.current_state["angular"] = req.angular_key
            self.current_state["linear"] = req.linear_key
            self.set_wheel_parameters();
            srv_ret = False
            try:
                if self.update_controller_srv():
                    res.success = True
                    srv_ret = True
                else:
                    srv_ret = False
                    res.success = False
                    res.message = "The kinematics on fts_controller failed to be updated!"
                    rospy.logerr(res.message)
                #if self.update_twist_controller_srv():
                    #res.success = True
                #else:
                    #res.success = False
                    #res.message = "The kinematics on twist_controller failed to be updated!"
                    #rospy.logerr(res.message)
                if self.update_odometry_controller_srv():
                    res.success = True
                    srv_ret = True
                else:
                    srv_ret = False
                    res.success = False
                    res.message = "The kinematics on odometry_controller failed to be updated!"
                    rospy.logerr(res.message)
            except rospy.ServiceException, e:
                res.success = False
                res.message = "The kinematics update service call failed!"
                rospy.logerr(res.message)

            # Return values to old ones if controller call not succeeded
            if not res.success:
                self.current_state = old_state.copy()
                self.set_wheel_parameters()

        res.angular = str(self.current_state["angular"]) + " - " + STATES_ANGULAR[self.current_state["angular"]][1]
        res.linear = str(self.current_state["linear"]) + " - " + STATES_LINEAR[self.current_state["linear"]][1]

        return res


    def get_rear_wheel_state(self, req):

        res = GetRearWheelsStateResponse()
        res.angular = str(self.current_state["angular"]) + " - " + STATES_ANGULAR[self.current_state["angular"]][1]
        res.linear = str(self.current_state["linear"]) + " - " + STATES_ANGULAR[self.current_state["linear"]][1]

        return res


    def read_set_default_wheel_parameters(self):
        try:
          wheel_params = rospy.get_param(self.wheel_controller_prefix)
          rospy.loginfo("Wheel parameters found at:" + self.wheel_controller_prefix)
        except KeyError, e:
          rospy.logerr("Key not found: " + str(e))
          rospy.logerr("The weeel parameters not be set on parameter server!")
          return

        for wheel in wheel_params:
            steer_pos = STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][1] * \
                        STATES_ANGULAR[self.current_state["angular"]][2]
            rospy.logwarn("Wheel: " + wheel["steer"] + " Setting 'wheel_angle_offset_to_plf_x' to: " +  str(steer_pos) + " as default.")
            wheel["wheel_angle_offset_to_plf_x"] = steer_pos

            #steer_neutral_pos = STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][0] + \
                        #STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][1] * STATES_ANGULAR[self.current_state["angular"]][2]
            #if not "steer_neutral_position" in wheel.keys():
              #rospy.logwarn("Wheel: " + wheel["steer"] + " 'steer_neutral_position' parameter not found. Setting to " + str(steer_neutral_pos) + " as default.")
              #STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][0] = steer_neutral_pos
              #wheel["steer_neutral_position"] = steer_neutral_pos

            #elif wheel["steer_neutral_position"] != steer_neutral_pos:
              #rospy.logwarn("Wheel: " + wheel["steer"] + " 'steer_neutral_position' parameter diferes from those defined in 'robotrainer_rear_wheels_state_publisher.py' node. Using value: " + steer_neutral_pos + " from the node.")
              #STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][0] = steer_neutral_pos

        rospy.set_param(self.wheel_controller_prefix, wheel_params);


    def set_wheel_parameters(self):
        try:
          wheel_params = rospy.get_param(self.wheel_controller_prefix)
          rospy.logdebug("Wheel parameters found at:" + self.wheel_controller_prefix)
        except KeyError, e:
          rospy.logerr("Key not found: " + str(e))
          rospy.logerr("The weeel parameters not be set on parameter server!")
          return

        for wheel in wheel_params:
            wheel["wheel_angle_offset_to_plf_x"] = STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][1] * \
                                                   STATES_ANGULAR[self.current_state["angular"]][2]
            #wheel["steer_neutral_position"] = STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][0] + \
                                              #STEER_WHEEL_NEUTRAL_POSITION[wheel["steer"]][1] * STATES_ANGULAR[self.current_state["angular"]][2]

        rospy.logerr("Setting wheel parameters:\n" + str(wheel_params));
        rospy.set_param(self.wheel_controller_prefix, wheel_params);


def main(args):
    robotrainer_state = RoboTrainerRearConfigurationStatePublisher()


if __name__ == "__main__":
    main(sys.argv)
