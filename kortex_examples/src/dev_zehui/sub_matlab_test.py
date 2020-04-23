#!/usr/bin/python

import roslib
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import sensor_msgs.msg
import sys, copy

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import numpy as np

DES_JOINTS_TOPIC = '/RTD/des_states'
NO_DOF = 7

class run_this_class(object):

    def __init__(self):
        self.trajectory = JointTrajectory()
        rospy.init_node('traj_sub')
        self.sub_des_states = rospy.Subscriber(DES_JOINTS_TOPIC, JointTrajectory, self.callback_des_states, tcp_nodelay=True, queue_size=1)
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

        self.trajectory.points = [JointTrajectoryPoint()]
        self.trajectory.joint_names = self.joint_names
        self.trajectory.points[0].positions = np.zeros(NO_DOF)
        self.trajectory.points[0].velocities = np.zeros(NO_DOF)
        self.trajectory.points[0].time_from_start = rospy.Duration(2.0)

        rospy.sleep(2.0)


    def callback_des_states(self, data):
        
        self.trajectory.points = data.points
        # self.trajectory.joint_names = self.joint_names


if __name__=='__main__':
    example = run_this_class()

    trajectory = example.trajectory

    print(trajectory.points[0])

    i = 0
    while i <  6001:
        if len(trajectory.points[i].positions) != 7:
            if len(trajectory.points[i].velocities) != 7:
                if len(trajectory.points[i].accelerations) != 7:
                    print("wrong!")
                    print(i)
        i += 1
