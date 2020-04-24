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
# Modified by Alexandre Vannobel to initialize simulated Kinova Gen3 robot in Gazebo 

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
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTolerance


DES_JOINTS_TOPIC = '/RTD/des_states'
TRAJ_EXE_TOPIC = '/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory'
NO_DOF = 7

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories"""
    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        rospy.init_node('example_move_it_trajectories')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            moveit_commander.roscpp_initialize(sys.argv)
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                    moveit_msgs.msg.DisplayTrajectory, queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                rospy.loginfo("gripper exists!!!")
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

        except Exception as e:
            rospy.logerr(e)
            self.is_init_success = False
        else:
            self.is_init_success = True


        # Added by Zehui Lu, Jan 21, 2020
        # self.sub_des_states = rospy.Subscriber(DES_JOINTS_TOPIC, JointTrajectory, self.callback_des_states, tcp_nodelay=True, queue_size=1)

        # self.client = actionlib.SimpleActionClient(TRAJ_EXE_TOPIC, FollowJointTrajectoryAction)
        # rospy.loginfo("actionlib begins")
        # self.client.wait_for_server()
        # self.client.cancel_goal()
        # rospy.loginfo("actionlib completed")
        # self.trajectory = JointTrajectory()

        # set up flag for new trajectory received
        # self.new_trajectory_received_flag = False

        # self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

        # self.trajectory = JointTrajectory()
        # self.trajectory.points = [JointTrajectoryPoint()]
        # self.trajectory.joint_names = self.joint_names
        # self.trajectory.points[0].positions = np.zeros(NO_DOF)
        # self.trajectory.points[0].velocities = np.zeros(NO_DOF)
        # self.trajectory.points[0].time_from_start = rospy.Duration(2.0)

        rospy.sleep(2.0)

        rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())


    def move_via_trajectory(self):
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = self.trajectory

        # print(self.trajectory.points[0])
        
        for jn in self.joint_names:
            goal_tol = JointTolerance()
            goal_tol.name = jn
            goal_tol.position = 0.01
            goal_tol.velocity = 0.01
            goal_tol.acceleration = 0.01
            follow_goal.goal_tolerance.append(goal_tol)

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()
        print("completed!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

        # print(self.client.get_result())

        self.new_trajectory_received_flag = False


    def callback_des_states(self, data):
        self.trajectory.points = data.points
        # self.trajectory.joint_names = self.joint_names
        self.new_trajectory_received_flag = True


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


    def get_joint_states(self):
        # Get the current joint positions

        joint_positions = self.arm_group.get_current_joint_values()
        return joint_positions

########################################################
    def home_the_robot(self):
        # Home the robot
        self.arm_group.set_named_target("home")
        return self.arm_group.go(wait=True)



def main():
    try:

        # For testing purposes
        try:
            rospy.delete_param("is_initialized")
        except:
            pass

        example = ExampleMoveItTrajectories()
        rospy.loginfo("Created example")
        success = example.is_init_success
        rospy.loginfo("success = {}".format(success))
        if success:
            success &= example.home_the_robot()

        # cmd_new = [0.0] * NO_DOF
        # example.zehui_reach_joint_angles(cmd_new, tolerance=0.01)
        # rospy.loginfo("Initial Configuration Completed!")

        # Get joint states
        # joint_states = example.get_joint_states()
        # print("joint states")
        # print(joint_states)
        # print("joint states completed")

        # rospy.sleep(2.0)

        # example.move_via_trajectory()
        # print("check this!!!!!!")
    except:
        success = False

    # For testing purposes
    rospy.set_param("is_initialized", success)
    if not success:
        rospy.logerr("The Gazebo script failed. :(")
    else:
        rospy.loginfo("The Gazebo script executed without fail. :P")


if __name__ == '__main__':
    main()
