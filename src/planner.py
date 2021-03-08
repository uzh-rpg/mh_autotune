#!/usr/bin/env python

#Copyright (C) 2021 Alessandro Saviolo

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import pandas as pd
import numpy as np

import utils

from quadrotor_msgs.msg import Trajectory, TrajectoryPoint


def parse(path, save_trajectory, z_offset):
  df = pd.read_csv(path, dtype=float, header=0, usecols=[i for i in range(1, 19)])
  trajectory = Trajectory()
  trajectory.points = [parsePoint(p, z_offset) for _, p in df.iterrows()]
  trajectory.header.frame_id = "world"
  trajectory.header.stamp = rospy.Time.now()
  trajectory.type = 1
  rospy.loginfo("Parser || Parsed Time-Optimal Trajectory || Size %d || Duration %2.4f" % (len(trajectory.points), df.iloc[-1]["t"]))
  if save_trajectory:
    utils.saveTrajectory(trajectory)
  return trajectory

def parsePoint(p, z_offset):
  q = np.array([p.loc["q_w"],p.loc["q_x"],p.loc["q_y"],p.loc["q_z"]])
  mass = 1.0
  u_sum = p.loc["u_1"] + p.loc["u_2"] + p.loc["u_3"] + p.loc["u_4"]
  acc = rotateVectorByQuat(u_sum/mass, q)
  point = TrajectoryPoint()
  point.time_from_start = rospy.Duration(p.loc["t"])
  point.pose.position.x = p.loc["p_x"]
  point.pose.position.y = p.loc["p_y"]
  point.pose.position.z = p.loc["p_z"]+z_offset
  point.pose.orientation.w = q[0]
  point.pose.orientation.x = q[1]
  point.pose.orientation.y = q[2]
  point.pose.orientation.z = q[3]
  point.velocity.linear.x = p.loc["v_x"]
  point.velocity.linear.y = p.loc["v_y"]
  point.velocity.linear.z = p.loc["v_z"]
  point.acceleration.linear.x = acc[0][0]
  point.acceleration.linear.y = acc[1][0]
  point.acceleration.linear.z = acc[2][0]-9.8066
  point.heading = 0.0
  point.heading_rate = 0.0
  point.heading_acceleration = 0.0
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

