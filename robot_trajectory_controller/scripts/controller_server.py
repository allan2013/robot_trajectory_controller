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
import geometry_msgs.msg
import trajectory_msgs.msg
from robot_trajectory_controller.srv import ServerCommand, ServerCommandRequest, ServerCommandResponse
from fanuc_msgs.srv import ReadSingleIO, ReadSingleIORequest, WriteSingleIO, WriteSingleIORequest

class ControllerServer():
    def __init__(self):
        rospy.init_node('robot_trajectory_controller', anonymous=True)
        self._service = rospy.Service('server_command', ServerCommand, self.execute_command)

        self._controller_name = '/arm_controller/follow_joint_trajectory'

        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        # Action Serverへ接続
        self._trajectory_client = None
        self.connect_servers()

        # MoveGroupへ接続
        self._robot = None
        self._scene = None
        self._commander = None
        self.connect_move_group()

        self._is_moving = False
        self._cur_position = []

        # define joint constraints
        # self.set_constraints()

        rospy.loginfo("Ready")

    def clear_constraints(self):
        self._commander.clear_path_constraints()

    def set_constraints(self):
        self.clear_constraints()
        constraints = moveit_msgs.msg.Constraints()
        j1 = moveit_msgs.msg.JointConstraint()
        j1.joint_name = 'J1'
        j1.position = 0.
        j1.tolerance_above = math.pi/4.
        j1.tolerance_below = math.pi/4.
        j1.weight = 1.
        constraints.joint_constraints.append(j1)
        j2 = moveit_msgs.msg.JointConstraint()
        j2.joint_name = 'J2'
        j2.position = 0.
        j2.tolerance_above = math.pi/4.
        j2.tolerance_below = math.pi/4.
        j2.weight = 1.
        constraints.joint_constraints.append(j2)
        j3 = moveit_msgs.msg.JointConstraint()
        j3.joint_name = 'J3'
        j3.position = 0.
        j3.tolerance_above = math.pi/4.
        j3.tolerance_below = math.pi/4.
        j3.weight = 1.
        constraints.joint_constraints.append(j3)
        j4 = moveit_msgs.msg.JointConstraint()
        j4.joint_name = 'J4'
        j4.position = 0.
        j4.tolerance_above = math.pi*120./180.
        j4.tolerance_below = math.pi*120./180.
        j4.weight = 1.
        constraints.joint_constraints.append(j4)
        j5 = moveit_msgs.msg.JointConstraint()
        j5.joint_name = 'J5'
        j5.position = 0.
        j5.tolerance_above = math.pi*120./180.
        j5.tolerance_below = math.pi*120./180.
        j5.weight = 1.
        constraints.joint_constraints.append(j5)
        self._commander.set_path_constraints(constraints)
    
    def read_single_io(self, address, type):
        rospy.wait_for_service('read_single_io')
        try:
            client = rospy.ServiceProxy('read_single_io', ReadSingleIO)
            req = ReadSingleIORequest()
            req.type = type
            req.address = address
            res = client(req)
            return res.value
        except rospy.ServiceException:
            rospy.loginfo("ReadIo Failed.")
            return None
    
    def write_single_io(self, address, val, type):
        rospy.wait_for_service('write_single_io')
        try:
            client = rospy.ServiceProxy('write_single_io', WriteSingleIO)
            req = WriteSingleIORequest()
            req.type = type
            req.address = address
            req.value = val
            client(req)
            return True
        except rospy.ServiceException:
            rospy.loginfo("ReadIo Failed.")
            return False

    def execute_command(self,req):
        rospy.loginfo(req)
        res = ServerCommandResponse()
        res.cmd_id = req.cmd_id
        res.is_moving = self._is_moving
        if req.type == ServerCommandRequest.GET_CURRENT_STATE:
            res.current_joints = self.get_current_joints()
            res.current_pose = self.get_current_ee_pose()
        elif req.type == ServerCommandRequest.MOVE_JOINT:
            if self.move_joints(req.target_joints) is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.MOVE_POSE:
            if self.move_pose(req.target_pose) is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.CANCEL_GOAL:
            if self.move_cancel() is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.GET_DI:
            res.io_value = self.read_single_io(req.io_address, ReadSingleIORequest.TYPE_DI)
            if res.io_value is None:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.GET_RI:
            res.io_value = self.read_single_io(req.io_address, ReadSingleIORequest.TYPE_RI)
            if res.io_value is None:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.SET_DO:
            if self.write_single_io(req.io_address, req.io_value, WriteSingleIORequest.TYPE_DO) is False:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.SET_RO:
            if self.write_single_io(req.io_address, req.io_value, WriteSingleIORequest.TYPE_RO) is False:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.SET_VELOCITY:
            self.set_speed(0, req.value)
        elif req.type == ServerCommandRequest.SET_ACCELRATION:
            self.set_speed(1, req.value)
        else:
            rospy.loginfo('Command not found.')
            res.cmd_id = -req.cmd_id

        return res

    def connect_move_group(self,robot_name='arm'):
        # MoveGroupへ接続
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._commander = moveit_commander.MoveGroupCommander(robot_name)
        self._commander.set_planner_id('RRTConnect')
        self._commander.set_planning_time(10)
        self._commander.set_num_planning_attempts(20)

    def connect_servers(self):
        timeout = rospy.Duration(10)
        self._trajectory_client = actionlib.SimpleActionClient(self._controller_name, FollowJointTrajectoryAction)
        if not self._trajectory_client.wait_for_server(timeout):
            print("Could not reach controller action. Make sure that the driver is actually running.")

    def joint_states_callback(self, msg):
        self.lock.acquire()
        self._cur_position = msg.position
        self.lock.release()
    
    def joint_states_listener(self):
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        #rospy.spin()

    def get_current_joints(self):
        return  self._cur_position
    
    def get_current_ee_pose(self):
        return self._commander.get_current_pose().pose

    def active_cb(self):
        self._is_moving = True
        rospy.loginfo("Action server is processing the goal")

    def feedback_cb(self,feedback):
        pass
        # rospy.loginfo(str(feedback))

    def done_cb(self,state,result):
        self._is_moving_ = False
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result.error_code)))

    def execute_plan(self):
        p = self._commander.plan()
        # print p

        if len(p.joint_trajectory.points) < 1:
            rospy.loginfo("Plan failed.")
            return False
        
        traj = p.joint_trajectory
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = traj.joint_names
        goal.trajectory.points = traj.points
        
        self._trajectory_client.send_goal(goal,
                            active_cb = self.active_cb,
                            feedback_cb = self.feedback_cb,
                            done_cb = self.done_cb
                            )

        rospy.loginfo("Goal has been sent to the action server.")
        if self._trajectory_client.wait_for_result():
            result = self._trajectory_client.get_result()
            if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
                rospy.loginfo("Execute plan failed: %s" % (result.error_string))
                return False
        else:
            return False

        return True

    def move_joints(self, tj):
        self._commander.set_joint_value_target(tj)
        return self.execute_plan()

    def move_pose(self, tp):
        self._commander.set_pose_target(tp)
        return self.execute_plan()

    def move_cancel(self):
        self._trajectory_client.cancel_all_goals()
        print("Canceled")

    def set_speed(self, type, speed):
        if (speed < 0.01):
            speed = 0.01
        if (speed > 1.0):
            speed = 1.0
        if (type == 0):
            self._commander.set_max_velocity_scaling_factor(speed)
        if (type == 1):
            self._commander.set_max_acceleration_scaling_factor(speed)

def main():
    cs = ControllerServer()
    rospy.spin()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
