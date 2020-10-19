#!/usr/bin/env python

import pandas as pd
import numpy as np
import rospy
import time
import os

from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import Trajectory, AutopilotFeedback

import planner


class Quadrotor():
  def __init__(self, z_offset, L_pivots):
    self.z_offset = z_offset
    self.L_pivots = L_pivots
    self.is_racing = False
    self.resources_path = rospy.get_param("/resources_path")
    self.file_name = rospy.get_param("/autotune_node/file_name")
    self.save_trajectory = rospy.get_param("/autotune_node/save_trajectory")
    self.distance_threshold = rospy.get_param("/autotune_node/distance_threshold")
    self.horizon_length = rospy.get_param("/autotune_node/horizon_length")
    self.trajectory = self.parseTrajectory()
    self.horizon_points = int(0.5 * self.horizon_length / (self.trajectory.points[1].time_from_start.nsecs * (10 ** -9))) + 1
    self.pivot_index = 0
    self.odometry_cnt = 0
    self.odometry_history = pd.DataFrame(columns=["x", "y", "z", "t"])
    self.trajectory_folder = time.strftime("%Y%m%d-%H%M%S")
    self.odometry_subscriber = rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.odometryCallback)
    self.state_subscriber = rospy.Subscriber("/hummingbird/autopilot/feedback", AutopilotFeedback, self.stateCallback)
    self.rviz_traj_publisher = rospy.Publisher("/rviz/trajectory", Path, queue_size=1)
    self.trajectory_publisher = rospy.Publisher("/hummingbird/autopilot/trajectory", Trajectory, queue_size=1)
    self.reset_mpc_publisher = rospy.Publisher("/hummingbird/autopilot/reset", Empty, queue_size=1)

  def run(self, parameter_list):
    rospy.loginfo("Drone || Run.")
    self.parameter_list = parameter_list
    self.setParameters()
    self.init()
    self.moveToStart()
    self.publishTrajectory()
    self.is_racing = True
    while self.autopilot_state != 2:
      if len(self.L_pivots) != self.pivot_index:
        self.trackTrajectory()
      rospy.rostime.wallsleep(0.1)
    rospy.loginfo("Drone || Finished racing!")
    saveStatistics(self.odometry_history, self.resources_path + "statistics/", self.file_name, self.trajectory_folder)
    return self.odometry_history.copy()

  def init(self):
    rospy.loginfo("Drone || Initialization.")
    publisher = rospy.Publisher("/hummingbird/bridge/arm", Bool, queue_size=1)
    wait_connection(publisher, rospy.Rate(200))
    msg = Bool()
    msg.data = True
    publisher.publish(msg)
    time.sleep(0.5)
    publisher = rospy.Publisher("/hummingbird/autopilot/start", Empty, queue_size=1)
    wait_connection(publisher, rospy.Rate(200))
    publisher.publish(Empty())
    while self.autopilot_state != 2:
      rospy.rostime.wallsleep(0.1)

  def reset(self):
    rospy.loginfo("Drone || Reset.")
    self.is_racing = False
    self.pivot_index = 0
    self.odometry_cnt = 0
    self.odometry_history = pd.DataFrame(columns=["x", "y", "z", "t"])
    resetPosition()    

  def moveToStart(self,):
    initial_pose = self.trajectory.points[0].pose
    rospy.loginfo("Drone || Moving to initial position || %2.2f %2.2f %2.2f" %
      (initial_pose.position.x, initial_pose.position.y, initial_pose.position.z))
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = initial_pose.position.x
    pose.pose.position.y = initial_pose.position.y
    pose.pose.position.z = initial_pose.position.z
    pose.pose.orientation.x = initial_pose.orientation.x
    pose.pose.orientation.y = initial_pose.orientation.y
    pose.pose.orientation.z = initial_pose.orientation.z
    pose.pose.orientation.w = initial_pose.orientation.w
    publisher = rospy.Publisher("/hummingbird/autopilot/pose_command", PoseStamped, queue_size=1)
    wait_connection(publisher, rospy.Rate(200))
    publisher.publish(pose)
    time.sleep(1.0)
    while self.autopilot_state != 2:
      rospy.rostime.wallsleep(0.1)
    time.sleep(5.0)

  def trackTrajectory(self):
#    rospy.loginfo("Drone || Tracking trajectory.")
    current_pivot = np.array([self.trajectory.points[self.L_pivots[self.pivot_index]].pose.position.x,
                              self.trajectory.points[self.L_pivots[self.pivot_index]].pose.position.y,
                              self.trajectory.points[self.L_pivots[self.pivot_index]].pose.position.z])
    current_odometry = np.array([self.odometry.pose.pose.position.x,
                                 self.odometry.pose.pose.position.y,
                                 self.odometry.pose.pose.position.z])
    if np.linalg.norm(current_odometry - current_pivot) <= self.distance_threshold:
      self.pivot_index += 1
      self.setParameters()

  def publishTrajectory(self):
#    rospy.loginfo("Test || Publishing trajectory.")
    wait_connection(self.trajectory_publisher, rospy.Rate(200))
    self.trajectory.header.stamp = rospy.Time.now()
    self.trajectory_publisher.publish(self.trajectory)
    time.sleep(1.0)

  def setParameters(self):
    rospy.loginfo("Drone || Updating parameters.")
    rospy.set_param("/hummingbird/autopilot/Q_pos_xy",    float(self.parameter_list[0, self.pivot_index]))
    rospy.set_param("/hummingbird/autopilot/Q_pos_z",     float(self.parameter_list[1, self.pivot_index]))
    rospy.set_param("/hummingbird/autopilot/Q_attitude",  float(self.parameter_list[2, self.pivot_index]))
    rospy.set_param("/hummingbird/autopilot/Q_velocity",  float(self.parameter_list[3, self.pivot_index]))
    rospy.set_param("/hummingbird/autopilot/Q_horizon",   float(self.parameter_list[4, self.pivot_index]))
    self.reset_mpc_publisher.publish(Empty())

  def stateCallback(self, state_msg):
#    rospy.loginfo("Drone || Updating state.")
    self.autopilot_state = state_msg.autopilot_state

  def odometryCallback(self, odometry_msg):
#    rospy.loginfo("Drone || Updating odometry.")
#    rospy.loginfo_throttle(0.5, "Drone || Position || %2.4f %2.4f %2.4f" %
#      (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z))
    self.odometry = odometry_msg
    if self.is_racing:
      self.odometry_history.loc[self.odometry_cnt] = [odometry_msg.pose.pose.position.x,
                                                      odometry_msg.pose.pose.position.y,
                                                      odometry_msg.pose.pose.position.z,
                                                      odometry_msg.header.stamp.secs + odometry_msg.header.stamp.nsecs * (10 ** -9)]
      self.odometry_cnt += 1

  def parseTrajectory(self):
    rospy.loginfo("Drone || Parsing trajectory.")
    return planner.parse(self.resources_path + "trajectories/" + self.file_name, self.save_trajectory, self.z_offset)


def resetPosition():
  os.system("rosservice call /gazebo/pause_physics")
  os.system("rosservice call /gazebo/unpause_physics")
  os.system("timeout 1s rostopic pub /hummingbird/autopilot/off std_msgs/Empty")
  os.system("rosservice call /gazebo/set_model_state "
            "'{model_state: { model_name: hummingbird, pose: { position: { x: 0.0, y: 0.0 ,z: 0.2 }, "
            "orientation: {x: 0, y: 0, z: 0.0, w: 1.0 }}, "
            "twist:{ linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 }}, "
            "reference_frame: world } }'")
  time.sleep(2.0)

def wait_connection(publisher, rate):
  while not publisher.get_num_connections():
    rate.sleep()

def saveStatistics(statistics, path, file_name, trajectory_folder):
  folder_path = path + file_name[:-4] + "/"
  if not os.path.exists(folder_path):
    os.makedirs(folder_path)
  folder_path += trajectory_folder + "/"
  if not os.path.exists(folder_path):
    os.makedirs(folder_path)
  statistics.to_csv(folder_path + time.strftime("%Y%m%d-%H%M%S") + ".csv", index=False)
  rospy.loginfo("Drone || Saved statistics.")

def toRosPath(trajectory, idx):
  path_msg = Path()
  t = rospy.Time.now()
  path_msg.header.stamp = t
  path_msg.header.frame_id = "world"
  for i in range(idx, len(trajectory.points)):
    p = trajectory.points[i]
    pose = PoseStamped()
    pose.header.stamp = t + p.time_from_start
    pose.pose.position.x = p.pose.position.x
    pose.pose.position.y = p.pose.position.y
    pose.pose.position.z = p.pose.position.z
    pose.pose.orientation.w = p.pose.orientation.w
    pose.pose.orientation.x = p.pose.orientation.x
    pose.pose.orientation.y = p.pose.orientation.y
    pose.pose.orientation.z = p.pose.orientation.z
    path_msg.poses.append(pose)
  return path_msg

if __name__ == '__main__':
  drone = Quadrotor()
  drone.run()

