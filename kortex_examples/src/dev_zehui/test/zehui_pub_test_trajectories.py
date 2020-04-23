#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

# Added by Zehui Lu, Jan 31, 2020
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


DES_JOINTS_TOPIC = "/RTD/des_states"
DES_TRAJ_TOPIC = "/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory/goal"
NO_DOF = 7

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):

        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", []) # it's empty []
        # self.gripper_joint_name = gripper_joint_names[0]
        self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

        # Create the MoveItInterface necessary objects
        arm_group_name = "arm"
        self.robot = moveit_commander.RobotCommander("robot_description")
        self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
        self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                    moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        # if self.is_gripper_present:
        #   gripper_group_name = "gripper"
        #   self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())


        # Added by Zehui Lu, Jan 21, 2020
        self.sub_des_states = rospy.Subscriber(DES_JOINTS_TOPIC, JointTrajectory, self.callback_des_states, tcp_nodelay=True, queue_size=1)
        self.new_trajectory_received_flag = False


        # set up flag for new trajectory received
        self.pub_traj = rospy.Publisher(DES_TRAJ_TOPIC, FollowJointTrajectoryAction)
        self.trajectory = JointTrajectory()
        

        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())


    def pub_traj_function(self):
        follow_goal = FollowJointTrajectoryAction()
        follow_goal.action_goal.goal.trajectory = self.trajectory
        self.new_trajectory_received_flag = False

        self.pub_traj.publish(follow_goal)

    def move_via_trajectory(self):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = self.trajectory
        self.new_trajectory_received_flag = False


    def callback_des_states(self, data):
        self.trajectory.points = data.points
        # self.trajectory.joint_names = self.joint_names
        self.new_trajectory_received_flag = True


    def reach_named_position(self, target):
        arm_group = self.arm_group
    
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        arm_group.execute(planned_path1, wait=True)


    def zehui_reach_joint_angles(self, cmd, tolerance):
        arm_group = self.arm_group

        # Get the current joint positions

        joint_positions = [0.0] * NO_DOF

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Read angle command [radian]

        # Set the joint target configuration
        for i in range(0, self.degrees_of_freedom):
            joint_positions[i] = cmd[i]
      

        rospy.loginfo("Joint Angle Example")

        arm_group.set_joint_value_target(joint_positions)
    
        # Plan and execute in one command

        arm_group.go(wait=True)
        # arm_group.go(wait=False)


    def zehui_get_joint_states(self):
        # Get the current joint positions

        joint_positions = self.arm_group.get_current_joint_values()
        return joint_positions



def main():
    example = ExampleMoveItTrajectories()


    example.reach_named_position("home")
    rospy.loginfo("Home Example Completed")

    # Get joint states
    joint_states = example.zehui_get_joint_states()
    print(joint_states)

    cmd = [0.0] * NO_DOF  # radian
    # cmd[6] = 0.2
    example.zehui_reach_joint_angles(cmd, tolerance=0.01)
    rospy.loginfo("Joint Angle Example Completed")


    # cmd_new = [0.0] * NO_DOF
    # example.zehui_reach_joint_angles(cmd_new, tolerance=0.01)
    # rospy.loginfo("Joint Angle Example Completed")


#########################################################
    t0 = time.time()
    print("start!")
    # example.pub_traj_function()
    print("completed!")
    t1 = time.time()
    dt = t1- t0
    print(dt)

    # print(example.trajectory.points[507])
    print("!!!!!!!!!!!!!!!!!!!!!!")


#########################################################





    # Calling ``stop()`` ensures that there is no residual movement

    # example.arm_group.stop()


if __name__ == '__main__':
    main()
