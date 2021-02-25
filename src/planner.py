#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np

#import utils

from agiros_msgs.msg import Reference, Setpoint, Command


def parse(path, save_trajectory=False, z_offset=0.0):
  df = pd.read_csv(path, dtype=float, header=0) #, usecols=[i for i in range(1, 19)]
  trajectory = Reference()
  trajectory.points = [parsePoint(p, z_offset) for _, p in df.iterrows()]
  trajectory.header.frame_id = "world"
  rospy.logwarn("Parser || Parsed Time-Optimal Trajectory || Size %d || Duration %2.4f" % (len(trajectory.points), df.iloc[-1]["t"]))
  #if save_trajectory:
  #  utils.saveTrajectory(trajectory)
  return trajectory

def parsePoint(p, z_offset):
  q = np.array([p.loc["q_w"],p.loc["q_x"],p.loc["q_y"],p.loc["q_z"]])
  mass = 1.0
  u_sum = p.loc["u_1"] + p.loc["u_2"] + p.loc["u_3"] + p.loc["u_4"]
  acc = rotateVectorByQuat(u_sum/mass, q)
  point = Setpoint()
  point.state.t = p.loc["t"]
  point.state.pose.position.x = p.loc["p_x"]
  point.state.pose.position.y = p.loc["p_y"]
  point.state.pose.position.z = p.loc["p_z"]+z_offset
  point.state.pose.orientation.w = q[0]
  point.state.pose.orientation.x = q[1]
  point.state.pose.orientation.y = q[2]
  point.state.pose.orientation.z = q[3]
  point.state.velocity.linear.x = p.loc["v_x"]
  point.state.velocity.linear.y = p.loc["v_y"]
  point.state.velocity.linear.z = p.loc["v_z"]
  point.state.velocity.angular.x = p.loc["w_x"]
  point.state.velocity.angular.y = p.loc["w_y"]
  point.state.velocity.angular.z = p.loc["w_z"]
  point.state.acceleration.linear.x = acc[0][0]
  point.state.acceleration.linear.y = acc[1][0]
  point.state.acceleration.linear.z = acc[2][0]-9.8066
  point.command.collective_thrust = u_sum
  point.command.bodyrates.x = p.loc["w_x"]
  point.command.bodyrates.y = p.loc["w_y"]
  point.command.bodyrates.z = p.loc["w_z"]
  return point

def rotateVectorByQuat(vz, q):
  Q, Q_bar = quatQ(q)
  v_rot_hom = np.matmul(
    np.matmul(Q_bar.transpose(), Q),
    np.array([[0], [0], [0], [vz]]))
  return v_rot_hom[1:4]

def quatQ(q):
  Q = np.array([[q[0], -q[1], -q[2], -q[3]],
                [q[1],  q[0], -q[3],  q[2]],
                [q[2],  q[3],  q[0], -q[1]],
                [q[3], -q[2],  q[1],  q[0]]])
  Q_bar = np.array([[q[0], -q[1], -q[2], -q[3]], 
                    [q[1],  q[0],  q[3], -q[2]], 
                    [q[2], -q[3],  q[0],  q[1]], 
                    [q[3],  q[2], -q[1],  q[0]]])
  return Q, Q_bar

