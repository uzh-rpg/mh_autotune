#!/usr/bin/env python

from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.font_manager import FontProperties
import matplotlib.pyplot as plt
import matplotlib
import os, sys
import numpy as np
import pandas as pd
import math
import seaborn as sns
import rospy
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

plt.rcParams['figure.figsize'] = (19.20, 10.80)
font = {'family' : 'sans',
        'weight' : 'normal',
        'size'   : 38}
matplotlib.rc('font', **font)
matplotlib.rcParams['pdf.fonttype'] = 42


class AutoTune():
  def __init__(self):
    self.parameters_path = rospy.get_param("/mpc_parameters")
    self.resources_path = rospy.get_param("/resources_path")
    self.file_name = rospy.get_param("/trajectory_file")
    self.iterations = rospy.get_param("/iterations")
    self.use_history = rospy.get_param("/use_history")
    self.drone = drone.Quadrotor(0.0, [], self.resources_path, self.file_name, self.parameters_path)

  def run(self):
    parameter_list = findBestParams(self.resources_path, self.parameters_path) if self.use_history else loadParams(self.parameters_path)
    histories, wp_errors, mean_errors, max_errors, sum_errors = [], [], [], [], []
    for it in range(self.iterations):
      history = self.drone.run(parameter_list, it)
      histories.append(history)
      wp_errors.append(computeWaypointErrors(history))
      sum_errors.append(history["error"].sum())
      mean_errors.append(history["error"].mean())
      max_errors.append(history["error"].max())
      cost = computeCost(history["error"])
      rospy.logwarn("")
      rospy.logwarn("AutoTune || Mean Error: %4.4f || Max Error: %4.4f || Sum Error: %4.4f || Cost: %4.4f" % (mean_errors[-1], max_errors[-1], sum_errors[-1], cost))
      rospy.logwarn("")
      self.drone.reset()
    mean_errors = np.array(mean_errors)
    max_errors = np.array(max_errors)
    sum_errors = np.array(sum_errors)
    rospy.logwarn("")
    rospy.logwarn("AutoTune || Finished diagnostic.")
    rospy.logwarn("AutoTune || Statistics over %i iterations:" % (self.iterations))
    rospy.logwarn("AutoTune || Mean Error: %4.4f" % (np.mean(mean_errors)))
    rospy.logwarn("AutoTune || Max Error: %4.4f" % (np.mean(max_errors)))
    rospy.logwarn("AutoTune || Sum Error: %4.4f" % (np.mean(sum_errors)))
    rospy.logwarn("")
    plotError(mean_errors, "mean", self.resources_path)
    plotError(max_errors, "max", self.resources_path)
    plotError(sum_errors, "sum", self.resources_path)
    for i, wp_error in enumerate(wp_errors):
      plotWaypointsError(wp_error, str(i), self.resources_path + "waypoints_error/")
    for i, h in enumerate(histories):
      plotGif(h, str(i), self.resources_path + "3dplot_error/")
    rospy.logwarn("")
    rospy.logwarn("AutoTune || Saved all statistics.")
    rospy.logwarn("")

def findBestParams(resources_path, parameters_path):
  df = pd.read_csv(resources_path + "history.csv", dtype=float, header=0)
  row = df.iloc[df["Error"].idxmin()]
  with open(parameters_path) as f:
      parameter_file = yaml.load(f)
      parameter_file["Q_pos_x"] = int(row["Q_pos_x_0"])
      parameter_file["Q_pos_y"] = int(row["Q_pos_y_0"])
      parameter_file["Q_pos_z"] = int(row["Q_pos_z_0"])
      parameter_file["Q_att_x"] = int(row["Q_att_x_0"])
      parameter_file["Q_att_y"] = int(row["Q_att_y_0"])
      parameter_file["Q_att_z"] = int(row["Q_att_z_0"])
      parameter_file["Q_vel"]   = int(row["Q_vel_0"])
      parameter_file["Q_omega"] = int(row["Q_omega_0"])
  with open(parameters_path, "w") as f:
    yaml.dump(parameter_file, f)
  return loadParams(parameters_path)

def loadParams(parameters_path):
  parameter_list = []
  with open(parameters_path) as f:
    parameter_file = yaml.load(f)
    parameter_list = [
      [parameter_file["Q_pos_x"]],
      [parameter_file["Q_pos_y"]],
      [parameter_file["Q_pos_z"]],
      [parameter_file["Q_att_x"]],
      [parameter_file["Q_att_y"]],
      [parameter_file["Q_att_z"]],
      [parameter_file["Q_vel"]],
      [parameter_file["Q_omega"]],
#      [parameter_file["Q_horizon"]]
    ]
#    parameter_list = [
#      [parameter_file["Q_pos_x"], parameter_file["Q_pos_x"]],
#      [parameter_file["Q_pos_y"], parameter_file["Q_pos_y"]],
#      [parameter_file["Q_pos_z"], parameter_file["Q_pos_z"]],
#      [parameter_file["Q_att_x"], parameter_file["Q_att_x"]],
#      [parameter_file["Q_att_y"], parameter_file["Q_att_y"]],
#      [parameter_file["Q_att_z"], parameter_file["Q_att_z"]],
#      [parameter_file["Q_vel"], parameter_file["Q_vel"]],
#      [parameter_file["Q_omega"], parameter_file["Q_omega"]],
#      [parameter_file["Q_horizon"], parameter_file["Q_horizon"]]
#    ]
  return np.array(parameter_list)

def plotError(error, error_type, path):
  fig = plt.figure()
  ax = fig.gca()
  ax.set_xlabel("iteration")
  ax.set_ylabel(error_type + " error (meters)")
  ax.plot(range(len(error)), error, color="red", alpha=1, linewidth=6)
  plt.grid()
  plt.savefig(path + error_type + ".png", format="png")

def plotWaypointsError(error, i, path):
  if not os.path.exists(path):
    os.makedirs(path)
  fig = plt.figure()
  ax = fig.gca()
  ax.set_xlabel("waypoint")
  ax.set_ylabel("error (meters)")
  ax.plot(range(len(error)), error, color="red", alpha=1, linewidth=6)
  plt.grid()
  plt.savefig(path + "iteration_" + i + ".png", format="png")

def computeWaypointErrors(df):
  wp = track.getRampUp()
  wp_indeces = [0] * len(wp)
  for k in range(len(wp)):
    smallest_distance = float("inf")
    for i, row in df.iterrows():
      if i < wp_indeces[k-1]:
        continue
      tmp_distance = np.linalg.norm(wp[k] - np.array([row["p_x"], row["p_y"], row["p_z"]]), ord=2)
      if tmp_distance < smallest_distance:
        smallest_distance = tmp_distance
        wp_indeces[k] = i
      elif tmp_distance > smallest_distance + 5.0:
        break
  wp_errors = [df["error"].iloc[i] for i in wp_indeces]
  return wp_errors

def plotGif(df, i, path):
  if not os.path.exists(path):
    os.makedirs(path)
  fig = plt.figure()
  ax = fig.gca(projection="3d")
  ax.set_xlabel("x (meters)", labelpad=40)
  ax.set_ylabel("y (meters)", labelpad=40)
  ax.set_zlabel("z (meters)", labelpad=40)
  df_error = (df["error"]-df["error"].min())/(df["error"].max()-df["error"].min())
  df_error *= 255.
  cmhot = plt.get_cmap("seismic")
  ax.scatter(df["p_x"], df["p_y"], df["p_z"], s=16, label="timeopt", c=df_error, cmap=cmhot)
  plt.tight_layout()
  angles = np.linspace(0, 360, 51)[:-1]
  rotanimate(ax, angles, path + "iteration_" + i + ".gif", delay=100)

def make_views(ax, angles, elevation=None, width=19.20, height=10.80, prefix="tmp_",**kwargs):
    files = []
    ax.figure.set_size_inches(width, height)
    for i,angle in enumerate(angles):
        ax.view_init(elev = elevation, azim=angle)
        fname = '%s%03d.png' % (prefix,i)
        ax.figure.savefig(fname)
        files.append(fname)
    return files
 
def make_movie(files,output, fps=10, bitrate=1800, **kwargs):
    output_name, output_ext = os.path.splitext(output)
    command = { '.mp4' : 'mencoder "mf://%s" -mf fps=%d -o %s.mp4 -ovc lavc\
                         -lavcopts vcodec=msmpeg4v2:vbitrate=%d'
                         %(",".join(files),fps,output_name,bitrate)}                    
    command['.ogv'] = command['.mp4'] + '; ffmpeg -i %s.mp4 -r %d %s'%(output_name,fps,output)
    print(command[output_ext])
    output_ext = os.path.splitext(output)[1]
    os.system(command[output_ext])

def make_gif(files,output,delay=100, repeat=True,**kwargs):
    loop = -1 if repeat else 0
    os.system('convert -delay %d -loop %d %s %s' % (delay,loop," ".join(files),output))
 
def make_strip(files,output,**kwargs):
    os.system('montage -tile 1x -geometry +0+0 %s %s' % (" ".join(files),output))
 
def rotanimate(ax, angles, output, **kwargs): 
    output_ext = os.path.splitext(output)[1]
    files = make_views(ax,angles, **kwargs)
    D = { '.mp4' : make_movie,
          '.ogv' : make_movie,
          '.gif': make_gif ,
          '.jpeg': make_strip,
          '.png':make_strip}
    D[output_ext](files,output,**kwargs)
    for f in files:
        os.remove(f)

def computeCost(history_error):
  return (history_error.apply(lambda x: np.exp(5*x)).sum())**3

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
#    q_horizon  = "Q_horizon    "
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
#      q_horizon  += "%8.2f  -->  %8.2f   " % (q[8][j], q_new[8][j])
    rospy.logwarn(s)
    rospy.logwarn(q_pos_x)
    rospy.logwarn(q_pos_y)
    rospy.logwarn(q_pos_z)
    rospy.logwarn(q_att_x)
    rospy.logwarn(q_att_y)
    rospy.logwarn(q_att_z)
    rospy.logwarn(q_vel)
    rospy.logwarn(q_omega)
#    rospy.logwarn(q_horizon)
    rospy.logwarn("\n")

def saveParameters(resources_path, score, parameters, mode="a", header=False):
  parameter_names = ["Q_pos_x", "Q_pos_y", "Q_pos_z", "Q_att_x", "Q_att_y", "Q_att_z", "Q_vel", "Q_omega"]    # , "Q_horizon"
  for i in range(len(parameters)):
    for j in range(len(parameters[i])):
      score[parameter_names[i] + "_" + str(j)] = parameters[i][j]
  score = pd.DataFrame([score])
  score.to_csv(resources_path + "history.csv", mode=mode, header=header)

def wait_connection(publisher, rate):
  while not publisher.get_num_connections():
    rate.sleep()

if __name__ == "__main__":
  rospy.init_node("autotune_node", anonymous=True)
  optimizer = AutoTune()
  optimizer.run()

