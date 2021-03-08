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

import drone
import track
import copy
import pivot
import regressor


class AutoTune():
  def __init__(self):
    rospy.init_node("autotune_node", anonymous=True)
    self.resources_path = rospy.get_param("/resources_path")
    self.file_name = rospy.get_param("/autotune_node/file_name")
    self.sigma = rospy.get_param("/autotune_node/sigma")
    self.max_iterations = rospy.get_param("/autotune_node/max_iterations")
    self.slow_velocity = rospy.get_param("/autotune_node/slow_velocity")
    self.early_stop = rospy.get_param("/autotune_node/early_stop")
    self.z_offset = rospy.get_param("/autotune_node/z_offset")
    self.bounds = {"0": rospy.get_param("/hummingbird/autopilot/B_pos_xy"),
                   "1": rospy.get_param("/hummingbird/autopilot/B_pos_z"),
                   "2": rospy.get_param("/hummingbird/autopilot/B_attitude"),
                   "3": rospy.get_param("/hummingbird/autopilot/B_velocity"),
                   "4": rospy.get_param("/hummingbird/autopilot/B_horizon")}
    self.parameter_list = regressor.predict(self.file_name[:-4], self.resources_path)
    self.L_pivots = pivot.getPivots(self.file_name)
    self.change_parameter = len(self.L_pivots)
    self.gates = loadGates(self.file_name[:-4], self.z_offset)
    self.num_gates = len(self.gates)
    self.gate_times = self.computeGateTimes()
    self.drone = drone.Quadrotor(self.z_offset, self.L_pivots)
    self.config = self.initConfiguration()
    self.config_best = self.config
    self.count_accepted = 0

  def run(self):
    for it in range(1, self.max_iterations):
      config_new = self.transitionModel()
      if self.acceptance(config_new):
        config_new["Accepted"] = 1
        self.config = config_new
        self.count_accepted += 1
        rospy.loginfo("Accepted configuration at iteration %d." % (it))
        rospy.loginfo("Acceptance rate: %2.2f %%." % (self.count_accepted*(1/it)*100))
        if config_new["Cost"] <= self.config_best["Cost"]:
          self.config_best = config_new
          rospy.loginfo("Updated best configuration.")
      else:
        rospy.loginfo("Rejected configuration at iteration %d." % (it))
      rospy.loginfo("Best configuration || Trajectory completion: %2.2f %% || Time: %4.4f || Cost: %4.4f" %
        (self.config_best["TrajectoryCompletion"], self.config_best["Time"], self.config_best["Cost"]))
    saveParameters(self.resources_path, self.config_best, self.parameter_list, mode="a", header=False)
    rospy.loginfo("Finished Racing.")

  def initConfiguration(self):
    odometry_history = self.drone.run(self.parameter_list)
    score = self.computeFinalTime(odometry_history)
    score["Cost"] = self.computeCost(score["Time"])
    rospy.loginfo("Trajectory completion: %2.2f %% || Time: %4.4f || Cost: %4.4f" %
      (score["TrajectoryCompletion"], score["Time"], score["Cost"]))
    saveParameters(self.resources_path, score, self.parameter_list, mode="w", header=True)
    return score

  def transitionModel(self):
    rospy.loginfo("Optimizing from segment %d." % (self.change_parameter))
    new_parameter_list = np.copy(self.parameter_list)
    rows = len(self.parameter_list)-1
    cols = len(self.parameter_list[0])
    for i in range(rows):
      for j in range(cols):
        if j == self.change_parameter - 3:
          new_parameter_list[i][j] = round(self.boundedNormal(str(i), new_parameter_list[i][j], self.sigma*0.25), 3)
        elif j == self.change_parameter - 2:
          new_parameter_list[i][j] = round(self.boundedNormal(str(i), new_parameter_list[i][j], self.sigma*0.50), 3)
        elif j == self.change_parameter - 1:
          new_parameter_list[i][j] = round(self.boundedNormal(str(i), new_parameter_list[i][j], self.sigma*0.75), 3)
        elif j == self.change_parameter:
          new_parameter_list[i][j] = round(self.boundedNormal(str(i), new_parameter_list[i][j], self.sigma), 3)
    for j in range(cols):
      if j == self.change_parameter - 3:
        new_parameter_list[rows][j] = round(self.boundedNormal(str(rows), new_parameter_list[rows][j], self.sigma*0.05), 3)
      elif j == self.change_parameter - 2:
        new_parameter_list[rows][j] = round(self.boundedNormal(str(rows), new_parameter_list[rows][j], self.sigma*0.05), 3)
      elif j == self.change_parameter - 1:
        new_parameter_list[rows][j] = round(self.boundedNormal(str(rows), new_parameter_list[rows][j], self.sigma*0.10), 3)
      elif j == self.change_parameter:
        new_parameter_list[rows][j] = round(self.boundedNormal(str(rows), new_parameter_list[rows][j], self.sigma*0.10), 3)
    rospy.loginfo("Updating Parameters List:")
    logParameterList(self.parameter_list, new_parameter_list)
    self.parameter_list = np.copy(new_parameter_list)
    self.drone.reset()
    odometry_history = self.drone.run(self.parameter_list)
    score = self.computeFinalTime(odometry_history)
    score["Cost"] = self.computeCost(score["Time"])
    rospy.loginfo("Trajectory completion: %2.2f %% || Time: %4.4f || Cost: %4.4f" %
      (score["TrajectoryCompletion"], score["Time"], score["Cost"]))
    saveParameters(self.resources_path, score, self.parameter_list, mode="a", header=False)
    return score

  def boundedNormal(self, parameter_name, value, sigma):
    new_value = np.random.normal(value, sigma)
    if new_value >= self.bounds[parameter_name][0] and new_value < self.bounds[parameter_name][1]:
      return new_value
    else:
      return self.boundedNormal(parameter_name, value, sigma)

  def computeFinalTime(self, history):
    elapsed_time = 0.0
    gate_times = copy.deepcopy(self.gate_times)
    for j in range(len(gate_times)):
      min_distance = float("inf")
      tmp_time = None
      tmp_idx = 0
      for i, row in history.iterrows():
        d = np.linalg.norm(self.gates[j+1] - np.array([row["x"], row["y"], row["z"]]), ord=2)
        if d < min_distance:
          min_distance = d
          tmp_time = row["t"]
          tmp_idx = i
      if min_distance > 1.2 and j != len(gate_times) - 1:
        self.change_parameter = len(self.L_pivots)
        for i in range(len(self.L_pivots)):
          if tmp_idx <= self.L_pivots[i]:
            self.change_parameter = i
            break
        for k in range(j, len(gate_times)):
          rospy.loginfo("Gate %3d || Elapsed Time: %8.4f || Gate Time: %8.4f || Error: %8.4f" % (k+1, elapsed_time, gate_times[k], min_distance))
        return {"Time": np.sum(gate_times), "TrajectoryCompletion": round((j * (1 / (len(self.gates)-2))) * 100, 3)}
      gate_times[j] = (tmp_time - history["t"].iloc[0]) - elapsed_time
      elapsed_time += gate_times[j]
      rospy.loginfo("Gate %3d || Elapsed Time: %8.4f || Gate Time: %8.4f || Error: %8.4f" % (j+1, elapsed_time, gate_times[j], min_distance))
    self.change_parameter = len(self.L_pivots)
    return {"Time": np.sum(gate_times), "TrajectoryCompletion": 100.0}

  def computeGateTimes(self):
    gate_times = [np.linalg.norm(self.gates[i+1] - self.gates[i]) / self.slow_velocity for i in range(self.num_gates - 2 - self.early_stop)]
    gate_times[0] += 3
    return gate_times

  def computeCost(self, t):
    return np.exp(2*np.sqrt(t))

  def acceptance(self, config_new):
    if config_new["Cost"] < self.config["Cost"]:
      return True
    else:
      accept = np.random.uniform(0, 1)
      return (accept < (self.config["Cost"]/config_new["Cost"]))


def loadGates(file_name, z_offset):
  gates = None
  if file_name == "circle":
    gates = track.getCircle()
  elif file_name == "spiral":
    gates = track.getSpiral()
  elif file_name == "drop":
    gates = track.getDrop()
  elif file_name == "flip":
    gates = track.getFlip()
  else:
    rospy.loginfo("AutoTune || Filename NOT correct.")
  for g in gates:
    g[2] += z_offset
  return gates

def logParameterList(q, q_new):
  l = range(len(q[0]))
  chunks = [list(l[i:i+4]) for i in range(0, len(l), 4)]
  for c in chunks:
    q_pos_xy   = "Q_pos_xy     "
    q_pos_z    = "Q_pos_z      "
    q_attitude = "Q_attitude   "
    q_velocity = "Q_velocity   "
    q_horizon  = "Q_horizon    "
    s = ""
    for j in c:
      s += "                         " + str(j)
      q_pos_xy   += "%8.2f  -->  %8.2f   " % (q[0][j], q_new[0][j])
      q_pos_z    += "%8.2f  -->  %8.2f   " % (q[1][j], q_new[1][j])
      q_attitude += "%8.2f  -->  %8.2f   " % (q[2][j], q_new[2][j])
      q_velocity += "%8.2f  -->  %8.2f   " % (q[3][j], q_new[3][j])
      q_horizon  += "%8.2f  -->  %8.2f   " % (q[4][j], q_new[4][j])
    rospy.loginfo(s)
    rospy.loginfo(q_pos_xy)
    rospy.loginfo(q_pos_z)
    rospy.loginfo(q_attitude)
    rospy.loginfo(q_velocity)
    rospy.loginfo(q_horizon)
    rospy.loginfo("\n")

def saveParameters(resources_path, score, parameters, mode="a", header=False):
  parameter_names = ["Q_pos_xy", "Q_pos_z", "Q_attitude", "Q_velocity", "Q_horizon"]
  for i in range(len(parameters)):
    for j in range(len(parameters[i])):
      score[parameter_names[i] + "_" + str(j)] = parameters[i][j]
  score = pd.DataFrame([score])
  score.to_csv(resources_path + "history.csv", mode=mode, header=header)

if __name__ == "__main__":
  optimizer = AutoTune()
  optimizer.run()

