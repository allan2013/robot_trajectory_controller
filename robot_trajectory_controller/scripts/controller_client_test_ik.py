#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from robot_trajectory_controller.srv import ServerCommand, ServerCommandRequest
from visualization_msgs.msg import InteractiveMarkerFeedback
import time
import math
import threading

#moving = False
moving = threading.Lock()

def handle(msg):
    global moving
    #if moving is True:
    #    rospy.loginfo("Skip")
    #    return
    #moving = True
    with moving:
        try:
            client = rospy.ServiceProxy('server_command', ServerCommand)
            req = ServerCommandRequest()
            req.type = ServerCommandRequest.MOVE_POSE_IK
            req.target_pose = msg.pose
            res = client(req)
        except rospy.ServiceException:
            rospy.loginfo("Fail.")
    #moving = False

def main():
    rospy.init_node('robot_trajectory_client_ik', anonymous=True)
    rospy.wait_for_service('server_command')

    sub = rospy.Subscriber("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", InteractiveMarkerFeedback, handle)
    rospy.spin()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
