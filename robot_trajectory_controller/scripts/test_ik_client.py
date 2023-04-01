#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import math
import rospy
import actionlib
import std_msgs.msg
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
    FollowJointTrajectoryFeedback,
    JointTolerance)
from sensor_msgs.msg import JointState
import threading
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
import geometry_msgs.msg
import trajectory_msgs.msg
from robot_trajectory_controller.srv import ServerCommand, ServerCommandRequest, ServerCommandResponse
from fanuc_msgs.srv import ReadSingleIO, ReadSingleIORequest, WriteSingleIO, WriteSingleIORequest
from pprint import pprint

rospy.init_node('compute_ik_test')

c = rospy.ServiceProxy('/compute_ik', GetPositionIK)
c.wait_for_service()

pose = geometry_msgs.msg.PoseStamped()
pose.pose.position.x = 0.6*100000
pose.pose.position.y = -0.15
pose.pose.position.z = 0.8

req = GetPositionIKRequest()
req.ik_request.group_name = 'arm'
req.ik_request.pose_stamped = pose
req.ik_request.timeout = rospy.Duration(1.0)
req.ik_request.attempts = 0
req.ik_request.avoid_collisions = True
try:
    resp = c.call(req)
    pprint(resp)
except rospy.ServiceException as e:
    rospy.logerr("Service exception: " + str(e))
    resp = GetPositionIKResponse()
    resp.error_code = 99999  # Failure
    pprint(resp)

