#!/usr/bin/env python

from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.metrics import mean_squared_error, explained_variance_score
from sklearn.decomposition import PCA
from sklearn import preprocessing
from itertools import combinations
import rospy
import numpy as np
import pandas as pd
import math
import csv
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)

import planner
import pivot


horizon_threshold = 100
height_threshold = 1
degree_threshold = 1
horizon_length = 0.6


def train(level, resources_path, feature_list):
  features = {
    "blue":   {"X": [], "y": []},
    "green":  {"X": [], "y": []},
    "red":    {"X": [], "y": []},
    "white":  {"X": [], "y": []}
  }
  with open(resources_path + "parameters.csv", "r") as f:
    reader = csv.reader(f, delimiter=";")
    for row in reader:
      if len(row) == 1: row = row[0].split(",")
      row = [r for r in row if r != ""]
      if row[0] == level: continue
      df = pd.read_csv(resources_path + "training_data/" + row[0] + ".csv", dtype=float, header=0)
      horizon_points = int(0.5 * horizon_length / df["t"][1]) + 1
      positions = df["p_z"].to_numpy()
      df_vel = df[["v_x", "v_y", "v_z"]].apply(np.linalg.norm, axis=1)
      df_slope = df["p_z"].diff(-10)
      df_thrust = df[["u_1", "u_2", "u_3", "u_4"]].apply(np.sum, axis=1)
      accelerations = {"f_x": [], "f_y": [], "f_z": []}
      for _, p in df.iterrows():
        q = np.array([p.loc["q_w"], p.loc["q_x"], p.loc["q_y"], p.loc["q_z"]])
        u_sum = p.loc["u_1"] + p.loc["u_2"] + p.loc["u_3"] + p.loc["u_4"]
        acc = planner.rotateVectorByQuat(u_sum, q)
        accelerations["f_x"].append(acc[0][0])
        accelerations["f_y"].append(acc[1][0])
        accelerations["f_z"].append(acc[2][0]-9.8066)
      df_acc = pd.DataFrame(accelerations)
      df_acc = df_acc.apply(np.linalg.norm, axis=1)
      idx = 1
      bounds = pivot.getBounds(positions, horizon_threshold, height_threshold, degree_threshold, horizon_points)
      for b in bounds:
        delta_x = b[1] - b[0]
        delta_y = df["p_z"][b[1]] - df["p_z"][b[0]]
        X = []
        for f in feature_list:
          if f == "delta_y":
            X.append(delta_y)
          elif f == "delta_x":
            X.append(delta_x)
          elif f == "initial_v":
            X.append(df_vel[b[0]])
          elif f == "initial_a":
            X.append(df_acc[b[0]])
          elif f == "initial_slope":
            X.append(df_slope[b[0]])
          elif f == "delta_v":
            X.append(df_vel[b[1]] - df_vel[b[0]])
          elif f == "delta_a":
            X.append(df_acc[b[1]] - df_acc[b[0]])
          elif f == "max_v":
            X.append(df_vel.loc[b[0]:b[1]].max())
          elif f == "max_a":
            X.append(df_acc.loc[b[0]:b[1]].max())
          elif f == "max_slope":
            X.append(df_slope.loc[b[0]:b[1]].max())
          elif f == "mean_v":
            X.append(df_vel.loc[b[0]:b[1]].mean())
          elif f == "mean_a":
            X.append(df_acc.loc[b[0]:b[1]].mean())
          elif f == "mean_slope":
            X.append(df_slope.loc[b[0]:b[1]].mean())
          elif f == "alpha":
            if b[2] == "blue" or b[2] == "green":
              X.append(math.degrees(math.atan(0.01 * horizon_points * (1 / (positions[b[0]] - positions[b[0] + horizon_points])))))
              X.append(math.degrees(math.atan(0.01 * delta_x / delta_y)))
            elif b[2] == "red":
              X.append(math.degrees(math.atan(0.01 * horizon_points * (1 / (positions[b[0] + horizon_points] - positions[b[0]])))))
              X.append(math.degrees(math.atan(0.01 * delta_x / delta_y)))
            else:
              X.append(0.0)
              X.append(0.0)
        features[b[2]]["X"].append([float(i) for i in X])
        y = [row[idx], row[idx+1], row[idx+2], row[idx+3], row[idx+4]]
        features[b[2]]["y"].append([float(i) for i in y])
        idx += 5
  for k in features.keys():
    features[k]["X"] = np.array(features[k]["X"])
  models = {}
  scalers = {}
  for k in features.keys():
    X = features[k]["X"]
    y = features[k]["y"]
    scalers[k] = preprocessing.Normalizer().fit(X)
    X = scalers[k].transform(X)
    gbr = GradientBoostingRegressor()
    model = MultiOutputRegressor(estimator=gbr)
    model.fit(X, y)
    models[k] = model
  return models, scalers

def infer(models, scalers, file_name, resources_path, feature_list):
  L_pos_xy, L_pos_z, L_attitude, L_velocity, L_horizon, L_omega = [], [], [], [], [], []
  df = pd.read_csv(resources_path + "trajectories/" + file_name + ".csv", dtype=float, header=0)
  horizon_points = int(0.5 * horizon_length / df["t"][1]) + 1
  positions = df["p_z"].to_numpy()
  df_vel = df[["v_x", "v_y", "v_z"]].apply(np.linalg.norm, axis=1)
  df_slope = df["p_z"].diff(-10)
  df_thrust = df[["u_1", "u_2", "u_3", "u_4"]].apply(np.sum, axis=1)
  accelerations = {"f_x": [], "f_y": [], "f_z": []}
  for _, p in df.iterrows():
    q = np.array([p.loc["q_w"], p.loc["q_x"], p.loc["q_y"], p.loc["q_z"]])
    u_sum = p.loc["u_1"] + p.loc["u_2"] + p.loc["u_3"] + p.loc["u_4"]
    acc = planner.rotateVectorByQuat(u_sum, q)
    accelerations["f_x"].append(acc[0][0])
    accelerations["f_y"].append(acc[1][0])
    accelerations["f_z"].append(acc[2][0]-9.8066)
  df_acc = pd.DataFrame(accelerations)
  df_acc = df_acc.apply(np.linalg.norm, axis=1)
  bounds = pivot.getBounds(positions, horizon_threshold, height_threshold, degree_threshold, horizon_points)
  for b in bounds:
    delta_x = b[1] - b[0]
    delta_y = df["p_z"][b[1]] - df["p_z"][b[0]]
    X = []
    for f in feature_list:
      if f == "delta_y":
        X.append(delta_y)
      elif f == "delta_x":
        X.append(delta_x)
      elif f == "initial_v":
        X.append(df_vel[b[0]])
      elif f == "initial_a":
        X.append(df_acc[b[0]])
      elif f == "initial_slope":
        X.append(df_slope[b[0]])
      elif f == "delta_v":
        X.append(df_vel[b[1]] - df_vel[b[0]])
      elif f == "delta_a":
        X.append(df_acc[b[1]] - df_acc[b[0]])
      elif f == "max_v":
        X.append(df_vel.loc[b[0]:b[1]].max())
      elif f == "max_a":
        X.append(df_acc.loc[b[0]:b[1]].max())
      elif f == "max_slope":
        X.append(df_slope.loc[b[0]:b[1]].max())
      elif f == "mean_v":
        X.append(df_vel.loc[b[0]:b[1]].mean())
      elif f == "mean_a":
        X.append(df_acc.loc[b[0]:b[1]].mean())
      elif f == "mean_slope":
        X.append(df_slope.loc[b[0]:b[1]].mean())
      elif f == "alpha":
        if b[2] == "blue" or b[2] == "green":
          X.append(math.degrees(math.atan(0.01 * horizon_points * (1 / (positions[b[0]] - positions[b[0] + horizon_points])))))
          X.append(math.degrees(math.atan(0.01 * delta_x / delta_y)))
        elif b[2] == "red":
          X.append(math.degrees(math.atan(0.01 * horizon_points * (1 / (positions[b[0] + horizon_points] - positions[b[0]])))))
          X.append(math.degrees(math.atan(0.01 * delta_x / delta_y)))
        else:
          X.append(0.0)
          X.append(0.0)
    X = np.array([X])
    X = scalers[b[2]].transform(X)
    prediction = list(models[b[2]].predict(X)[0])
    L_pos_xy.append(round(float(prediction[0]), 3))
    L_pos_z.append(round(float(prediction[1]), 3))
    L_attitude.append(round(float(prediction[2]), 3))
    L_velocity.append(round(float(prediction[3]), 3))
    L_horizon.append(round(float(prediction[4]), 3))
    L_omega.append(30.0)
  return L_pos_xy, L_pos_z, L_attitude, L_velocity, L_omega, L_horizon

def predict(level, resources_path):
#  feature_list = ["delta_y","delta_x","delta_v","delta_a","mean_slope"]
#  models, scalers = train(level, resources_path, feature_list)
#  L_pos_xy, L_pos_z, L_attitude, L_velocity, L_omega, L_horizon = infer(models, scalers, level, resources_path, feature_list)

  L_pos_xy   =  [  500.0  ]
  L_pos_z    =  [  700.0  ]
  L_attitude =  [   10.0  ]
  L_velocity =  [    5.0  ]
  L_omega  =    [  30.00  ]
  L_horizon  =  [   6.00  ]

  return np.array([L_pos_xy, L_pos_z, L_attitude, L_velocity, L_omega, L_horizon])

