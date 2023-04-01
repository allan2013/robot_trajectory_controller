#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from robot_trajectory_controller.srv import ServerCommand, ServerCommandRequest
import time
import math

def main():
    rospy.init_node('robot_trajectory_client', anonymous=True)
    rospy.wait_for_service('server_command')
    try:
        client = rospy.ServiceProxy('server_command', ServerCommand)
        req = ServerCommandRequest()
        req.cmd_id = 1
        req.type = ServerCommandRequest.GET_CURRENT_STATE
        robot_state = client(req)
        rospy.loginfo(robot_state)

        req.cmd_id = 99
        req.type = ServerCommandRequest.SET_VELOCITY
        req.value = rospy.get_param('~vel', 0.8)
        res = client(req)

        req.type = ServerCommandRequest.GET_RI
        req.io_address = 1
        res = client(req)
        print(res.io_value)

        req.type = ServerCommandRequest.SET_RO
        req.io_address = 1
        req.io_value = 0
        res = client(req)

        val = rospy.get_param('~rotate', 60.0)
        print(val)
        val *= math.pi/180.
        
        req.type = ServerCommandRequest.MOVE_JOINT
        tj = list(robot_state.current_joints)
        tj[0] += val
        req.target_joints = tj
        req.cmd_id = 3
        res = client(req)

        time.sleep(0.1)
        req.type = ServerCommandRequest.GET_CURRENT_STATE
        robot_state = client(req)
        rospy.loginfo(res.is_moving)

        flag = rospy.get_param('~cancel', True)
        print(flag)
        
        if flag:
            time.sleep(0.3)
            req.type = ServerCommandRequest.CANCEL_GOAL
            client(req)

        time.sleep(3)
        req.type = ServerCommandRequest.GET_CURRENT_STATE
        robot_state = client(req)
        rospy.loginfo(robot_state.current_joints)

    except rospy.ServiceException:
        rospy.loginfo("Fail.")

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
