#!/usr/bin/env python

import os, sys
import numpy as np
import pandas as pd
import math
import rospy
import time
import yaml

if __name__ == "__main__":
  rospy.init_node("autotune_node", anonymous=True)
  parameters_path = rospy.get_param("/mpc_parameters")
  resources_path = rospy.get_param("/resources_path")
  df = pd.read_csv(resources_path + "history.csv", dtype=float, header=0)
  row = df.iloc[df["Error"].idxmin()]
  print(row)
  print(row["Q_pos_x_0"])

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

