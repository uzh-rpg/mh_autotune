#!/usr/bin/env python

import pandas as pd
import numpy as np
import rospy
import time
import os
import copy
import yaml
from datetime import datetime
import time

from agiros_msgs.msg import QuadState, Reference, Telemetry
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import planner


class Quadrotor():
  def __init__(self, z_offset, L_pivots, resources_path, file_name, parameters_path):
    self.z_offset = z_offset
    self.L_pivots = L_pivots
    self.resources_path = resources_path
    self.file_name = file_name
    self.parameters_path = parameters_path
    self.horizon_length = rospy.get_param("/autotune_node/horizon_length")
    self.distance_threshold = rospy.get_param("/autotune_node/distance_threshold")
    self.trajectory_time = rospy.get_time()
    self.trajectory_folder = time.strftime("%Y%m%d-%H%M%S")
    self.trajectory = self.parseTrajectory()
    self.history = pd.DataFrame(columns=["error", "t", "p_x", "p_y", "p_z"])
    self.pivot_index = 0
    self.history_cnt = 0
    self.odometry_t = 0
    self.reference = []
    self.odometry = []
    self.current_time = rospy.get_time()
    self.is_racing = False
    self.is_hovering = False
    self.state_subscriber = rospy.Subscriber("/hummingbird/agiros_pilot/telemetry", Telemetry, self.stateCallback)
    self.odometry_subscriber = rospy.Subscriber("/hummingbird/agiros_pilot/state", QuadState, self.odometryCallback)
    self.odometry_subscriber = rospy.Subscriber("/hummingbird/active/path", Path, self.referenceCallback)
    self.trajectory_publisher = rospy.Publisher("/hummingbird/agiros_pilot/trajectory", Reference, queue_size=1)
    self.reset_mpc_publisher = rospy.Publisher("/hummingbird/agiros_pilot/reset", Empty, queue_size=1)
    self.reset_publisher = rospy.Publisher("/hummingbird/agiros_pilot/reset_sim", Empty, queue_size=1)
    self.off_publisher = rospy.Publisher("/hummingbird/agiros_pilot/off", Empty, queue_size=1)
    self.move_to_start_publisher = rospy.Publisher("/hummingbird/agiros_pilot/go_to_pose", PoseStamped, queue_size=1)

  def run(self, parameter_list, it):
    rospy.logwarn("Drone || Run.")
    self.parameter_list = parameter_list
    self.setParameters()
    self.init()
    self.moveToStart()
    self.publishTrajectory()
    self.is_racing = True
    while not self.is_hovering:
#      if len(self.L_pivots) != self.pivot_index:
#      self.trackTrajectory()
      rospy.rostime.wallsleep(0.1)
    self.is_racing = False
    self.history = self.history[:-15]
    saveStatistics(self.history, self.resources_path+"statistics/", self.file_name, self.trajectory_folder, it)
    rospy.logwarn("Drone || Finished racing!")
    return self.history.copy()

  def init(self):
    rospy.logwarn("Drone || Taking off.")
    publisher = rospy.Publisher("/hummingbird/agiros_pilot/enable", Bool, queue_size=1)
    wait_connection(publisher, rospy.Rate(200))
    msg = Bool()
    msg.data = True
    publisher.publish(msg)
    rospy.rostime.wallsleep(0.5)
    publisher = rospy.Publisher("/hummingbird/agiros_pilot/start", Empty, queue_size=1)
    wait_connection(publisher, rospy.Rate(200))
    publisher.publish(Empty())
    rospy.rostime.wallsleep(0.5)
    while not self.is_hovering:
      rospy.logwarn_throttle(1.0, "Drone || Initialization || Waiting for hover state.")
      rospy.rostime.wallsleep(0.1)
    rospy.logwarn("Drone || Took off successfully.")

  def moveToStart(self):
    initial_pose = self.trajectory.points[0].state.pose
    rospy.logwarn("Drone || Moving to initial position || %2.2f %2.2f %2.2f" % (initial_pose.position.x, initial_pose.position.y, initial_pose.position.z))
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = initial_pose.position.x
    pose.pose.position.y = initial_pose.position.y
    pose.pose.position.z = initial_pose.position.z
    pose.pose.orientation.x = initial_pose.orientation.x
    pose.pose.orientation.y = initial_pose.orientation.y
    pose.pose.orientation.z = initial_pose.orientation.z
    pose.pose.orientation.w = initial_pose.orientation.w
    wait_connection(self.move_to_start_publisher, rospy.Rate(200))
    self.move_to_start_publisher.publish(pose)
    rospy.rostime.wallsleep(0.5)
    while not self.is_hovering:
      rospy.logwarn_throttle(1.0, "Drone || Waiting to reach hover state.")
      rospy.rostime.wallsleep(0.1)
    rospy.logwarn("Drone || Moved to initial position successfully.")

  def publishTrajectory(self):
    rospy.logwarn("Drone || Publishing trajectory.")
    wait_connection(self.trajectory_publisher, rospy.Rate(200))
    self.trajectory.header.stamp = rospy.Time.now()
    self.current_time = rospy.get_time()
    for point in self.trajectory.points:
      point.state.t += self.current_time
      point.command.t += self.current_time
    self.trajectory_publisher.publish(self.trajectory)
    while self.is_hovering:
      rospy.logwarn_throttle(1.0, "Drone || Waiting to leave hover state.")
      rospy.rostime.wallsleep(0.1)
    rospy.logwarn("Drone || Published trajectory successfully.")

  def trackTrajectory(self):
#    rospy.logwarn("Drone || Tracking trajectory.")
    current_pivot = np.array([self.trajectory.points[self.L_pivots[self.pivot_index]].state.pose.position.x,
                              self.trajectory.points[self.L_pivots[self.pivot_index]].state.pose.position.y,
                              self.trajectory.points[self.L_pivots[self.pivot_index]].state.pose.position.z])
    if np.linalg.norm(self.odometry-current_pivot) <= self.distance_threshold:
      self.pivot_index = 1 if self.pivot_index == 0 else 0
      self.setParameters()

  def reset(self):
    rospy.logwarn("Drone || Resetting simulation.")
    self.off_publisher.publish(Empty())
    time.sleep(1.0)
    self.reset_publisher.publish(Empty())
    time.sleep(2.0)
    for point in self.trajectory.points:
      point.state.t -= self.current_time
      point.command.t -= self.current_time
    self.is_racing = False
    self.is_hovering = False
    self.pivot_index = 0
    self.history_cnt = 0
    self.history = pd.DataFrame(columns=["error", "t", "p_x", "p_y", "p_z"])
    rospy.logwarn("Drone || Reset simulation successfully.")

  def parseTrajectory(self):
    rospy.logwarn("Drone || Parsing trajectory.")
    return planner.parse(self.resources_path+"trajectories/"+self.file_name, self.z_offset)

  def setParameters(self):
    rospy.logwarn("Drone || Updating parameters.")
    with open(self.parameters_path) as f:
      parameter_file = yaml.load(f)
      parameter_file["Q_pos_x"] = int(self.parameter_list[0, self.pivot_index])
      parameter_file["Q_pos_y"] = int(self.parameter_list[1, self.pivot_index])
      parameter_file["Q_pos_z"] = int(self.parameter_list[2, self.pivot_index])
      parameter_file["Q_att_x"] = int(self.parameter_list[3, self.pivot_index])
      parameter_file["Q_att_y"] = int(self.parameter_list[4, self.pivot_index])
      parameter_file["Q_att_z"] = int(self.parameter_list[5, self.pivot_index])
      parameter_file["Q_vel"] = int(self.parameter_list[6, self.pivot_index])
      parameter_file["Q_omega"] = int(self.parameter_list[7, self.pivot_index])
#      parameter_file["Q_horizon"] = int(self.parameter_list[8, self.pivot_index])
    with open(self.parameters_path, "w") as f:
      yaml.dump(parameter_file, f)
    wait_connection(self.reset_mpc_publisher, rospy.Rate(200))
    self.reset_mpc_publisher.publish(Empty())

  def stateCallback(self, state_msg):
#    rospy.logwarn("Drone || Updating state.")
#    rospy.logwarn_throttle(1.0, "Drone || State || %10.2f" % state_msg.reference_left_duration)
    self.is_hovering = True if state_msg.reference_left_duration == float("inf") else False
    if self.is_racing:
      error = np.linalg.norm(self.reference - self.odometry, ord=1)
      self.history.loc[self.history_cnt] = [error, self.odometry_t, self.odometry[0], self.odometry[1], self.odometry[2]]
      self.history_cnt += 1
#      rospy.logwarn_throttle(0.25, "Drone || Tracking error || %3.3f" % error)

  def odometryCallback(self, odometry_msg):
#    rospy.logwarn("Drone || Updating odometry.")
#    rospy.logwarn_throttle(0.5, "Drone || Drone odometry || %2.4f %2.4f %2.4f" %
#      (odometry_msg.pose.position.x, odometry_msg.pose.position.y, odometry_msg.pose.position.z))
    self.odometry = np.array([odometry_msg.pose.position.x, odometry_msg.pose.position.y, odometry_msg.pose.position.z])
    self.odometry_t = odometry_msg.t

  def referenceCallback(self, reference_msg):
#    rospy.logwarn("Drone || Updating reference.")
#    rospy.logwarn_throttle(0.5, "Drone || Reference odometry || %2.4f %2.4f %2.4f" %
#      (reference_msg.poses[0].pose.position.x, reference_msg.poses[0].pose.position.y, reference_msg.poses[0].pose.position.z))
    self.reference = np.array([reference_msg.poses[0].pose.position.x, reference_msg.poses[0].pose.position.y, reference_msg.poses[0].pose.position.z])

def logParameters():
  param_names = rospy.get_param_names()
  for param_name in param_names:
    print(param_name, rospy.get_param(param_name))

def wait_connection(publisher, rate):
  while not publisher.get_num_connections():
    rospy.logwarn_throttle(1.0, "Drone || Waiting for a listener.")
    rate.sleep()

def saveStatistics(statistics, path, file_name, trajectory_folder, it):
  folder_path = path + file_name[:-4] + "/"
  if not os.path.exists(folder_path):
    os.makedirs(folder_path)
  folder_path += trajectory_folder + "/"
  if not os.path.exists(folder_path):
    os.makedirs(folder_path)
  statistics.to_csv(folder_path + "iteration_" + str(it) + ".csv", index=False)
  rospy.logwarn("Drone || Saved statistics.")

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

