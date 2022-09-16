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
        req.type = ServerCommandRequest.GET_CURRENT_STATE
        robot_state = client(req)
        rospy.loginfo(robot_state.current_joints)
        rospy.loginfo(robot_state.current_pose)

        req.type = ServerCommandRequest.GET_DI
        req.io_address = 1
        res = client(req)
        print(res.io_value)

        req.type = ServerCommandRequest.SET_DO
        req.io_address = 1
        req.io_value = 1
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
        print(res.cmd_id)

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
