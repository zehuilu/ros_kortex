#!/usr/bin/python


import roslib
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import sensor_msgs.msg
import sys, copy
from cubic_hermite_spline import TCubicHermiteSpline


def InterpolateTraj(q_traj, t_traj, dt=1.0e-3):
  if t_traj[0]>1.0e-6:
    raise Exception('t_traj[0] must be zero and q_traj[0] must be the current joint angles.')
  assert(len(q_traj)==len(t_traj))

  #Modeling the trajectory with spline.
  DoF= len(q_traj[0])
  splines= [TCubicHermiteSpline() for d in range(DoF)]
  for d in range(DoF):
    data_d= [[t,q[d]] for q,t in zip(q_traj,t_traj)]
    splines[d].Initialize(data_d, tan_method=splines[d].CARDINAL, c=0.0, m=0.0)

  traj_points= []
  t= 0.0
  while t<t_traj[-1]:
    q,v,a= [],[],[]
    for spline in splines:
      q_d,v_d,a_d= spline.Evaluate(t,with_dd=True)
      q.append(q_d)
      v.append(v_d)
      a.append(a_d)
    point= trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions= q
    point.velocities= v
    point.accelerations= a
    point.time_from_start= rospy.Duration(t)
    traj_points.append(point)
    t+= dt
  #JTP= trajectory_msgs.msg.JointTrajectoryPoint
  #traj_points= np.array([JTP(*(np.array([[spline.Evaluate(t,with_dd=True)] for spline in splines]).T.tolist()+[[],rospy.Duration(t)]))
                         #for t in np.arange(0.0,t_traj[-1],dt)])
  return traj_points


if __name__=='__main__':
  rospy.init_node('gen3_test')

  joint_names= ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']

  client= actionlib.SimpleActionClient('/gen3a/gen3_joint_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)
  client.cancel_goal()  #Ensure to cancel the ongoing goal.

  # Wait some seconds for the head action server to start or exit
  if not client.wait_for_server(rospy.Duration(5.0)):
    rospy.logerr('Exiting - Joint Trajectory Action Server Not Found')
    rospy.signal_shutdown('Action Server not found')
    sys.exit(1)

  angles= rospy.wait_for_message('/gen3a/joint_states', sensor_msgs.msg.JointState, 5.0).position
  #WARNING: This may not work:
  #  We already reached the goal position : nothing to do.
  #  https://github.com/Kinovarobotics/ros_kortex/issues/29
  #But we can quickly fix this issue by following this:
  #  https://github.com/Kinovarobotics/ros_kortex/issues/29#issuecomment-558633653
  q_traj=[
    angles,
    [q+0.02 for q in angles],
    [q-0.02 for q in angles],
    angles]
  t_traj= [0.0, 1.0, 3.0, 4.0]
  #q_traj=[
    #angles,
    #[q-0.02 for q in angles],
    #[q+0.02 for q in angles]]
  #t_traj= [0.0, 1.0, 3.0]

  goal= control_msgs.msg.FollowJointTrajectoryGoal()
  for jn in joint_names:
    goal_tol= control_msgs.msg.JointTolerance()
    goal_tol.name= jn
    goal_tol.position= 0.01
    goal.goal_tolerance.append(goal_tol)
  goal.trajectory.joint_names= joint_names
  #t0=rospy.Time.now()
  goal.trajectory.points= InterpolateTraj(q_traj, t_traj)
  #print 'Calculation time [ms]:',(rospy.Time.now()-t0).to_sec()*1.0e+3
  goal.trajectory.header.stamp= rospy.Time.now()
  client.send_goal(goal)
  #client.cancel_goal()
  #client.wait_for_result(timeout=rospy.Duration(20.0))

  print client.get_result()

  #rospy.signal_shutdown('Done.')