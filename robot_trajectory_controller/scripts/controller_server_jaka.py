#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import math
import rospy
import actionlib
import tf
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
#from fanuc_msgs.srv import ReadSingleIO, ReadSingleIORequest, WriteSingleIO, WriteSingleIORequest
from jaka_msgs.srv import SetIO, SetIORequest, SetIOResponse, GetIO, GetIORequest, GetIOResponse
import copy
import numpy as np

class ControllerServer():
    def __init__(self):
        rospy.init_node('robot_trajectory_controller', anonymous=True)
        self._service = rospy.Service('server_command', ServerCommand, self.execute_command)
        self._state_pub = rospy.Publisher('server_state', std_msgs.msg.Int16, latch=True)
        self._state_pub.publish(std_msgs.msg.Int16(0))

        self._tf_listener = tf.TransformListener()

        self._controller_name = '/jaka_zu5_controller/follow_joint_trajectory'

        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        # 存在検知装置をmonitorするthread
        #self.thread_estop_monitor = threading.Thread(target=self.estop_monitor)
        #self.thread_estop_monitor.start()
        #self.human_detect_enable = True
        #rospy.Subscriber("human_detect_enable", std_msgs.msg.Bool, self.human_detect_enable_callback)

        # Action Serverへ接続
        self._trajectory_client = None
        self.connect_servers()

        # MoveGroupへ接続
        self._robot = None
        self._scene = None
        self._commander = None
        self.velocity_scaling_factor = 0.3
        self.acceleration_scaling_factor = 0.3
        self.connect_move_group()

        # Connect to IK server
        self._compute_ik_client = None
        self.connect_ik_server()

        self._is_moving = False
        self._cur_position = []
        self._skipped_goal = None

        # define path constraints
        #self.set_constraints()

        rospy.loginfo("Ready")

    def clear_constraints(self):
        self._commander.clear_path_constraints()

    def set_constraints(self):
        self.clear_constraints()
        constraints = moveit_msgs.msg.Constraints()
        c = moveit_msgs.msg.OrientationConstraint()
        c.header.frame_id = 'base_link'
        c.link_name = 'flange'
        c.orientation = geometry_msgs.msg.Quaternion(0., 0., 0., 1.)
        c.absolute_x_axis_tolerance = math.pi / 2.
        c.absolute_y_axis_tolerance = math.pi / 2.
        c.absolute_z_axis_tolerance = math.pi
        c.weight = 1.0
        constraints.orientation_constraints.append(c)
        #j1 = moveit_msgs.msg.JointConstraint()
        #j1.joint_name = 'J1'
        #j1.position = 0.
        #j1.tolerance_above = math.pi/4.
        #j1.tolerance_below = math.pi/4.
        #j1.weight = 1.
        #constraints.joint_constraints.append(j1)
        #j2 = moveit_msgs.msg.JointConstraint()
        #j2.joint_name = 'J2'
        #j2.position = 0.
        #j2.tolerance_above = math.pi/4.
        #j2.tolerance_below = math.pi/4.
        #j2.weight = 1.
        #constraints.joint_constraints.append(j2)
        #j3 = moveit_msgs.msg.JointConstraint()
        #j3.joint_name = 'J3'
        #j3.position = 0.
        #j3.tolerance_above = math.pi/4.
        #j3.tolerance_below = math.pi/4.
        #j3.weight = 1.
        #constraints.joint_constraints.append(j3)
        #j4 = moveit_msgs.msg.JointConstraint()
        #j4.joint_name = 'J4'
        #j4.position = 0.
        #j4.tolerance_above = math.pi*120./180.
        #j4.tolerance_below = math.pi*120./180.
        #j4.weight = 1.
        #constraints.joint_constraints.append(j4)
        #j5 = moveit_msgs.msg.JointConstraint()
        #j5.joint_name = 'J5'
        #j5.position = 0.
        #j5.tolerance_above = math.pi*120./180.
        #j5.tolerance_below = math.pi*120./180.
        #j5.weight = 1.
        #constraints.joint_constraints.append(j5)
        self._commander.set_path_constraints(constraints)
    
    def read_single_io(self, index, type):
        rospy.wait_for_service('read_single_io')
        try:
            client = rospy.ServiceProxy('read_single_io', GetIO)
            req = GetIORequest()
            req.type = type
            req.index = index
            req.signal = 0
            req.path = 0
            res = client(req)
            return res.value
        except rospy.ServiceException:
            rospy.loginfo("ReadIo Failed.")
            return None
    
    def write_single_io(self, index, val, type):
        rospy.wait_for_service('jaka_driver/set_io')
        try:
            # client = rospy.ServiceProxy('write_single_io', WriteSingleIO)
            # req = WriteSingleIORequest()
            # req.type = type
            # req.address = address
            # req.value = val
            # client(req)
            # return True

            client = rospy.ServiceProxy('jaka_driver/set_io', SetIO)
            req = SetIORequest()
            req.signal = 'digital'
            req.index = index
            req.type = type
            req.value = val
            res = client(req)
            print("----------")
            print(res)
            return res.ret


        except rospy.ServiceException as e:
            print(e)
            rospy.loginfo("write_single_io Failed.")
            return False

    def human_detect_enable_callback(self, m):
        self.human_detect_enable = m.data
        rospy.loginfo("Human detect enable: " + str(self.human_detect_enable))

    def estop_monitor(self):
        rospy.wait_for_service('read_single_io')
        client = rospy.ServiceProxy('read_single_io', ReadSingleIO)
        rospy.wait_for_service('write_single_io')
        wclient = rospy.ServiceProxy('write_single_io', WriteSingleIO)
        prev_red = 0
        prev_yellow = 0
        while True:
            try:
                req = ReadSingleIORequest()
                req.type = ReadSingleIORequest.TYPE_DI
                req.address = 110
                yellow = client(req).value
                req.address = 108
                red = client(req).value
                if red == 1 and prev_red == 0:
                    prev_red = 1
                    wreq = WriteSingleIORequest()
                    wreq.type = WriteSingleIORequest.TYPE_DO
                    wreq.address = 101
                    wreq.value = 0
                    wclient(wreq)
                    wreq.address = 102
                    wreq.value = 1
                    wclient(wreq)
                    wreq.address = 103
                    wreq.value = 0
                    wclient(wreq)
                    #wreq.address = 104
                    #wreq.value = 1
                    #wclient(wreq)
                elif yellow == 1 and prev_yellow == 0:
                    if self.human_detect_enable is True:
                        self.move_cancel()
                        self.set_speed(0, 0.05)
                    prev_yellow = 1
                    wreq = WriteSingleIORequest()
                    wreq.type = WriteSingleIORequest.TYPE_DO
                    wreq.address = 101
                    wreq.value = 1
                    wclient(wreq)
                    wreq.address = 102
                    wreq.value = 0
                    wclient(wreq)
                    wreq.address = 103
                    wreq.value = 0
                    wclient(wreq)
                    #wreq.address = 104
                    #wreq.value = 0
                    #wclient(wreq)
                elif yellow == 0 and red == 0 and (prev_yellow == 1 or prev_red == 1):
                    prev_yellow = 0
                    prev_red = 0
                    wreq = WriteSingleIORequest()
                    wreq.type = WriteSingleIORequest.TYPE_DO
                    wreq.address = 101
                    wreq.value = 1
                    wclient(wreq)
                    wreq.address = 102
                    wreq.value = 1
                    wclient(wreq)
                    wreq.address = 103
                    wreq.value = 1
                    wclient(wreq)
                    #wreq.address = 104
                    #wreq.value = 0
                    #wclient(wreq)
            except rospy.ServiceException as e:
                rospy.loginfo(e)
            rospy.sleep(0.1)

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
        elif req.type == ServerCommandRequest.PLAN_JOINT:
            if self.move_joints(req.target_joints, False) is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.PLAN_POSE:
            if self.move_pose(req.target_pose, False) is False:
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
            if self.write_single_io(req.io_address, req.io_value, 0) is False:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.SET_RO:
            if self.write_single_io(req.io_address, req.io_value, 0) is False:
                res_cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.SET_VELOCITY:
            self.set_speed(0, req.value)
        elif req.type == ServerCommandRequest.SET_ACCELERATION:
            self.set_speed(1, req.value)
        elif req.type == ServerCommandRequest.MOVE_POSE_IK:
            if self.move_pose_ik(req.target_pose) is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.PICK:
            if self.pick() is False:
                res.cmd_id = -req.cmd_id
        elif req.type == ServerCommandRequest.PREPICK:
            if self.pick(p = '/pre') is False:
                res.cmd_id = -req.cmd_id
        else:
            rospy.loginfo('Command not found.')
            res.cmd_id = -req.cmd_id

        return res

    def connect_move_group(self,robot_name='jaka_zu5'):
        # MoveGroupへ接続
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._commander = moveit_commander.MoveGroupCommander(robot_name)
        self._commander.set_planner_id('RRTConnect')
        self._commander.set_planning_time(10)
        self._commander.set_num_planning_attempts(20)
        #self._commander.set_num_planning_attempts(5)

    def connect_servers(self):
        timeout = rospy.Duration(10)
        self._trajectory_client = actionlib.SimpleActionClient(self._controller_name, FollowJointTrajectoryAction)
        if not self._trajectory_client.wait_for_server(timeout):
            print("Could not reach controller action. Make sure that the driver is actually running.")

    def connect_ik_server(self):
        self._compute_ik_client = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self._compute_ik_client.wait_for_service()

    def compute_ik(self, pose):
        req = GetPositionIKRequest()
        req.ik_request.group_name = 'jaka_zu5'
        req.ik_request.pose_stamped.header.frame_id = '/base_link'
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.timeout = rospy.Duration(1.0)
        req.ik_request.attempts = 10
        req.ik_request.avoid_collisions = False # True
        try:
            resp = self._compute_ik_client.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp

    def move_pose_ik(self, pose):
        p = self.compute_ik(pose)
        #print p

        if len(p.solution.joint_state.position) < 1:
            rospy.loginfo("Plan failed.")
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        tp = trajectory_msgs.msg.JointTrajectoryPoint()
        tp.positions = self._cur_position
        goal.trajectory.points.append(tp)
        tp2 = copy.deepcopy(tp)
        tp2.positions = p.solution.joint_state.position
        tp2.time_from_start.nsecs = 100
        goal.trajectory.points.append(tp2)
        #print goal

        if self._is_moving is True:
            rospy.loginfo("Moving")
            self._skipped_goal = goal
            return True

        self._trajectory_client.send_goal(goal,
                            active_cb = self.active_cb,
                            feedback_cb = self.feedback_cb,
                            done_cb = self.done_cb
                            )

        #rospy.loginfo("Goal has been sent to the action server.")
        #if self._trajectory_client.wait_for_result():
        #    result = self._trajectory_client.get_result()
        #    if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
        #        rospy.loginfo("Execute plan failed: %s" % (result.error_string))
        #        return False
        #else:
        #    return False

        return True

    def joint_states_callback(self, msg):
        self.lock.acquire()
        self._joint_names = msg.name
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
        self._state_pub.publish(std_msgs.msg.Int16(1))
        rospy.loginfo("Action server is processing the goal")

    def feedback_cb(self,feedback):
        pass
        # rospy.loginfo(str(feedback))

    def done_cb(self,state,result):
        self._is_moving = False
        self._state_pub.publish(std_msgs.msg.Int16(0))
        rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result.error_code)))
        if self._skipped_goal:
            self._trajectory_client.send_goal(self._skipped_goal,
                                              active_cb=self.active_cb,
                                              feedback_cb=self.feedback_cb,
                                              done_cb=self.done_cb)
            self._skipped_goal = None

    def vel_check(self, traj):
        vels_limit = np.array([1.57, 1.57, 1.57, 1.57, 1.57, 1.57])
        if len(traj.points) < 1:
            rospy.loginfo("traj is too short.")
            return False
        elif len(traj.points) == 2:
            print(traj)

        first = True
        prev_point = traj.points[0]
        for point in traj.points:
            # point = trajectory_msgs.msg.JointTrajectoryPoint()
            if first:
                first = False
                continue

            delta_sec = point.time_from_start.to_sec() - prev_point.time_from_start.to_sec()
            vels = (np.array(point.positions) - np.array(prev_point.positions)) / delta_sec
            if not min(vels <= vels_limit):
                rospy.loginfo("traj's vel limit is not protected.")
                print(point.positions)
                print(prev_point.positions)
                print(delta_sec)
                print(vels)
                return False

            prev_point = point

        return True


    def execute_plan(self, execute=True):
        p = self._commander.plan()
        #print p

        # while len(p.joint_trajectory.points) < 1:
        while not self.vel_check(p.joint_trajectory):
            rospy.loginfo("Re-planning.")
            p = self._commander.plan()
            rospy.timer.sleep(0.1)

        if execute is False:
            return True
        
        #traj = p.joint_trajectory
        rospy.loginfo("Retime trajectory with scaling factor: " + str(self.velocity_scaling_factor))
        traj = self._commander.retime_trajectory(self._robot.get_current_state(), p, self.velocity_scaling_factor).joint_trajectory
        # traj = self.scale_plan_vel(traj)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = traj.joint_names
        goal.trajectory.points = traj.points
        
        self._skipped_goal = None
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

    def scale_plan_vel(self, traj, spd_ratio):
        #print "scale_plan_vel!"
        points_num = len(traj.points)
        assert points_num > 0

        new_traj = copy.deepcopy(traj)
        for i in range(points_num):
            new_traj.points[i].time_from_start = \
                traj.points[i].time_from_start / spd_ratio
            vel = [v * spd_ratio for v in traj.points[i].velocities]
            acc = [a * spd_ratio for a in traj.points[i].accelerations]
            new_traj.points[i].velocities = tuple(vel)
            new_traj.points[i].accelerations = tuple(acc)
        return new_traj

    def move_joints(self, tj, execute=True):
        self._commander.set_joint_value_target(tj)
        return self.execute_plan(execute)

    def move_pose(self, tp, execute=True):
        self._commander.set_pose_reference_frame('base_link')
        self._commander.set_pose_target(tp)
        return self.execute_plan(execute)



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
            self.velocity_scaling_factor = speed
        if (type == 1):
            self._commander.set_max_acceleration_scaling_factor(speed)
            self.acceleration_scaling_factor = speed

    def pick(self, p = '/'):
        self._commander.set_pose_reference_frame('base_link')
        targets = []
        for t in ('pick1', 'pick2', 'pick3', 'pick4'):
            print p + t
            try:
                (trans, rot) = self._tf_listener.lookupTransform('/base_link', p + t, rospy.Time(0))
                print trans + rot
                targets.append(trans + rot)
            except Exception as e:
                print "lookup error"
                print e
                pass
        print targets
        if len(targets) == 0:
            return False
        self._commander.set_pose_targets(targets)
        return self.execute_plan(True)

def main():
    cs = ControllerServer()
    rospy.spin()

if __name__ =='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
