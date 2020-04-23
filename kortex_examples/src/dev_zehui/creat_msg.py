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
from sensor_msgs.msg import JointState

import generate_spline_peak_speed as gen_traj
import numpy as np
from math import pi, degrees, radians

import matplotlib
import matplotlib.pyplot as plt

DES_JOINTS_TOPIC = '/RTD/des_states'
JOINT_STATES_TOPIC = '/my_gen3/joint_states'
NO_DOF = 7

class creat_msg_class(object):
    def __init__(self):
        rospy.init_node('traj_gen')
        self.pub_traj = rospy.Publisher(DES_JOINTS_TOPIC, JointTrajectory, queue_size=1)
        self.sub_joint_states = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.callback_joint_states, queue_size=1)
        self.joint_position = JointState()

        print("Sleep begins")
        rospy.sleep(2.0)
        print("Sleep Completed")

    def callback_joint_states(self, data):
        self.joint_position = data.position[0 : 7]


def load_traj_as_ros_msg(traj_full):
    # load the full trajectory as a dictionary
    t_traj = traj_full["time_traj"]
    positions = traj_full["positions"]
    velocities = traj_full["velocities"]
    accelerations = traj_full["accelerations"]

    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    # trajectory.header.stamp= rospy.Time.now()
    traj_points= []

    # point.positions = [0.0] * NO_DOF
    # point.velocities = [0.0] * NO_DOF
    # point.accelerations = [0.0] * NO_DOF
    # point.time_from_start = rospy.Duration(2.0)


    i = 0

    while i < positions.shape[1]:

        point = JointTrajectoryPoint()
        point.positions = positions[:, i].tolist()

        point.velocities = velocities[:, i].tolist()
        point.accelerations = accelerations[:, i].tolist()

        # point.velocities = []
        # point.accelerations = []

        if i == 0:
            point.time_from_start = rospy.Duration(0.0)
        else:
            point.time_from_start = rospy.Duration(t_traj[i])
        traj_points.append(point)

        i += 1

    trajectory.points = traj_points

    return trajectory


def main(v_peak_full, t_peak, t_total, dt):
    msg_class = creat_msg_class()
    joint_states_now = msg_class.joint_position

    print(joint_states_now)

    traj_full = gen_traj.main(joint_states_now, v_peak_full, t_peak, t_total, dt)

    trajectory = load_traj_as_ros_msg(traj_full)

    msg_class.pub_traj.publish(trajectory)
    print("publish completed")


    positions = traj_full["positions"]
    velocities = traj_full["velocities"]
    accelerations = traj_full["accelerations"]

    print("maximum [radians]")
    print(np.amax(positions))
    print(np.amax(velocities))
    print(np.amax(accelerations))

    print("maximum [degrees]")
    print(degrees(np.amax(positions)))
    print(degrees(np.amax(velocities)))
    print(degrees(np.amax(accelerations)))

    print("initial joint angles [radians]")
    print(trajectory.points[0].positions)

    print("final joint angles [radians]")
    print(trajectory.points[-1].positions)

    print("angle difference [radians]")
    diff = np.array(trajectory.points[-1].positions) - np.array(trajectory.points[0].positions)
    print(diff)


##################################################
    # plot_flag = True
    plot_flag = False

    if plot_flag == True:
        time_plot = np.linspace(0.0, t_total, t_total/dt+1).tolist()

        fig_1, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.plot(time_plot, positions[4, :].tolist(), color="red", linestyle="-", label="angle")
        ax1.legend(loc="upper left")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("angle [radian]")

        ax2.plot(time_plot, velocities[4, :].tolist(), color="red", linestyle="-", label="velocity")
        ax2.legend(loc="upper left")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("velocity [radian/s]")

        ax3.plot(time_plot, accelerations[4, :].tolist(), color="red", linestyle="-", label="acceleration")
        ax3.legend(loc="upper left")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("acceleration [radian/s^2]")
        plt.tight_layout()


        plt.show()
##################################################



    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        msg_class.pub_traj.publish(trajectory)
        print("publish completed")
        rate.sleep()

    

if __name__=='__main__':
    v_peak_full = [0.0, 0.0, radians(8.5), 0.0, 0.0, 0.0, radians(8.5)]
    t_peak = 13
    t_total = 26
    dt = 0.001

    print("v_peak [radians/sec]")
    print(v_peak_full)

    main(v_peak_full, t_peak, t_total, dt)