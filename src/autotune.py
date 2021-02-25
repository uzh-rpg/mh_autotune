#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np
import time
import yaml

import drone
import planner
import track
import copy
import pivot
import regressor

from std_msgs.msg import Empty, Bool
from agiros_msgs.msg import Reference
from geometry_msgs.msg import PoseStamped


class AutoTune():
  def __init__(self):
    self.execution_idx = str(rospy.get_param("/execution_idx"))
    self.parameters_path = rospy.get_param("/mpc_parameters")
    self.resources_path = rospy.get_param("/resources_path")
    node_name = "/autotune_node_" + self.execution_idx
    self.file_name = rospy.get_param(node_name + "/file_name")
    self.sigma = rospy.get_param(node_name + "/sigma")
    self.max_iterations = rospy.get_param(node_name + "/max_iterations")
    self.slow_velocity = rospy.get_param(node_name + "/slow_velocity")
    self.early_stop = rospy.get_param(node_name + "/early_stop")
    self.z_offset = rospy.get_param(node_name + "/z_offset")
    self.bounds = {"0": rospy.get_param(node_name + "/B_pos_x"),
                   "1": rospy.get_param(node_name + "/B_pos_y"),
                   "2": rospy.get_param(node_name + "/B_pos_z"),
                   "3": rospy.get_param(node_name + "/B_att_x"),
                   "4": rospy.get_param(node_name + "/B_att_y"),
                   "5": rospy.get_param(node_name + "/B_att_z"),
                   "6": rospy.get_param(node_name + "/B_vel"),
                   "7": rospy.get_param(node_name + "/B_omega")} #,
                   #"8": rospy.get_param(node_name + "/B_horizon")}
    self.parameter_list = loadParams(self.parameters_path)
    self.L_pivots = pivot.getPivots(self.file_name)
    self.change_parameter = len(self.L_pivots)
    self.gates = loadGates(self.file_name[:-4], self.z_offset)
    self.num_gates = len(self.gates)
    self.gate_times = self.computeGateTimes()
    self.drone = drone.Quadrotor(self.z_offset, self.L_pivots, self.resources_path, self.file_name, self.parameters_path, node_name)
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
        rospy.logwarn("AutoTune || Accepted configuration at iteration %d." % (it))
        rospy.logwarn("AutoTune || Acceptance rate: %2.2f %%." % (self.count_accepted*(1/it)*100))
        if config_new["Cost"] <= self.config_best["Cost"]:
          self.config_best = config_new
          rospy.logwarn("AutoTune || Updated best configuration.")
      else:
        rospy.logwarn("AutoTune || Rejected configuration at iteration %d." % (it))
      rospy.logwarn("AutoTune || Best configuration || Error: %4.4f || Cost: %4.4f" % (self.config_best["Error"], self.config_best["Cost"]))
    saveParameters(self.resources_path, self.config_best, self.parameter_list, self.execution_idx, mode="a", header=False)
    rospy.logwarn("AutoTune || Finished Racing.")

  def initConfiguration(self):
    history = self.drone.run(self.parameter_list)
    error = history["error"].apply(lambda x: np.exp(5*x)).sum()
    score = {"Error": error, "Cost": self.computeCost(error)}
    rospy.logwarn("AutoTune || Error: %4.4f || Cost: %4.4f" % (score["Error"], score["Cost"]))
    saveParameters(self.resources_path, score, self.parameter_list, self.execution_idx, mode="w", header=True)
    return score

  def transitionModel(self):
    rospy.logwarn("AutoTune || Optimizing from segment %d." % (self.change_parameter))
    new_parameter_list = np.copy(self.parameter_list)
    rows = len(self.parameter_list)-1
    cols = len(self.parameter_list[0])
    for i in range(rows):
      for j in range(cols):
        new_parameter_list[i][j] = round(self.boundedNormal(str(i), new_parameter_list[i][j], self.sigma), 3)
    for j in range(cols):
      new_parameter_list[rows][j] = round(self.boundedNormal(str(rows), new_parameter_list[rows][j], self.sigma*0.10), 3)
    rospy.logwarn("AutoTune || Updating Parameters List:")
    logParameterList(self.parameter_list, new_parameter_list)
    self.parameter_list = np.copy(new_parameter_list)
    self.drone.reset()
    history = self.drone.run(self.parameter_list)
    error = history["error"].apply(lambda x: np.exp(5*x)).sum()
    score = {"Error": error, "Cost": self.computeCost(error)}
    rospy.logwarn("AutoTune || Error: %4.4f || Cost: %4.4f" % (score["Error"], score["Cost"]))
    saveParameters(self.resources_path, score, self.parameter_list, self.execution_idx, mode="a", header=False)
    return score

  def boundedNormal(self, parameter_name, value, sigma):
    new_value = np.random.normal(value, sigma)
    if new_value >= self.bounds[parameter_name][0] and new_value < self.bounds[parameter_name][1]:
      return new_value
    else:
      return self.boundedNormal(parameter_name, value, sigma)

  def computeGateTimes(self):
    gate_times = [np.linalg.norm(self.gates[i+1]-self.gates[i])/self.slow_velocity for i in range(self.num_gates-2-self.early_stop)]
    gate_times[0] += 3
    return gate_times

  def computeCost(self, t):
    return t**2 #np.exp(2*np.sqrt(t))

  def acceptance(self, config_new):
    if config_new["Cost"] < self.config["Cost"]:
      return True
    else:
      accept = np.random.uniform(0, 1)
      return (accept < (self.config["Cost"]/config_new["Cost"]))

def loadParams(parameters_path):
  parameter_list = []
  with open(parameters_path) as f:
    parameter_file = yaml.load(f)
    parameter_list = [
      [parameter_file["Q_pos_x"], parameter_file["Q_pos_x"]],
      [parameter_file["Q_pos_y"], parameter_file["Q_pos_y"]],
      [parameter_file["Q_pos_z"], parameter_file["Q_pos_z"]],
      [parameter_file["Q_att_x"], parameter_file["Q_att_x"]],
      [parameter_file["Q_att_y"], parameter_file["Q_att_y"]],
      [parameter_file["Q_att_z"], parameter_file["Q_att_z"]],
      [parameter_file["Q_vel"], parameter_file["Q_vel"]],
      [parameter_file["Q_omega"], parameter_file["Q_omega"]]
      #[parameter_file["Q_horizon"], parameter_file["Q_horizon"]]
    ]
  return np.array(parameter_list)

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
    gates = track.getRampUp()
  for g in gates:
    g[2] += z_offset
  return gates

def logParameterList(q, q_new):
  l = range(len(q[0]))
  chunks = [list(l[i:i+8]) for i in range(0, len(l), 8)]
  for c in chunks:
    q_pos_x    = "Q_pos_x      "
    q_pos_y    = "Q_pos_y      "
    q_pos_z    = "Q_pos_z      "
    q_att_x    = "Q_att_x      "
    q_att_y    = "Q_att_y      "
    q_att_z    = "Q_att_z      "
    q_vel      = "Q_vel        "
    q_omega    = "Q_omega      "
    #q_horizon  = "Q_horizon    "
    s = ""
    for j in c:
      s += "                         " + str(j)
      q_pos_x    += "%8.2f  -->  %8.2f   " % (q[0][j], q_new[0][j])
      q_pos_y    += "%8.2f  -->  %8.2f   " % (q[1][j], q_new[1][j])
      q_pos_z    += "%8.2f  -->  %8.2f   " % (q[2][j], q_new[2][j])
      q_att_x    += "%8.2f  -->  %8.2f   " % (q[3][j], q_new[3][j])
      q_att_y    += "%8.2f  -->  %8.2f   " % (q[4][j], q_new[4][j])
      q_att_z    += "%8.2f  -->  %8.2f   " % (q[5][j], q_new[5][j])
      q_vel      += "%8.2f  -->  %8.2f   " % (q[6][j], q_new[6][j])
      q_omega    += "%8.2f  -->  %8.2f   " % (q[7][j], q_new[7][j])
      #q_horizon  += "%8.2f  -->  %8.2f   " % (q[8][j], q_new[8][j])
    rospy.logwarn(s)
    rospy.logwarn(q_pos_x)
    rospy.logwarn(q_pos_y)
    rospy.logwarn(q_pos_z)
    rospy.logwarn(q_att_x)
    rospy.logwarn(q_att_y)
    rospy.logwarn(q_att_z)
    rospy.logwarn(q_vel)
    rospy.logwarn(q_omega)
    #rospy.logwarn(q_horizon)
    rospy.logwarn("\n")

def saveParameters(resources_path, score, parameters, execution_idx, mode="a", header=False):
  parameter_names = ["Q_pos_x", "Q_pos_y", "Q_pos_z", "Q_att_x", "Q_att_y", "Q_att_z", "Q_vel", "Q_omega"] #, "Q_horizon"]
  for i in range(len(parameters)):
    for j in range(len(parameters[i])):
      score[parameter_names[i] + "_" + str(j)] = parameters[i][j]
  score = pd.DataFrame([score])
  score.to_csv(resources_path + "history_" + execution_idx + ".csv", mode=mode, header=header)

def wait_connection(publisher, rate):
  while not publisher.get_num_connections():
    rate.sleep()

if __name__ == "__main__":
  rospy.init_node("autotune_node", anonymous=True)
  optimizer = AutoTune()
  optimizer.run()

