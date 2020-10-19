import matplotlib.pyplot as plt
import rospy
import numpy as np
import pandas as pd
import math


def getPivots(file_name, horizon_threshold=102, height_threshold=3, degree_threshold=1, horizon_length=0.6):
  file_name = file_name
  folder_path = rospy.get_param("/resources_path") + "trajectories/"
  df = pd.read_csv(folder_path+file_name, dtype=float, header=0)
  delta_x = int(0.5 * horizon_length / df["t"][1]) + 1
  bounds = getBounds(df["p_z"].to_numpy(), horizon_threshold, height_threshold, degree_threshold, delta_x)[1:]
  return [b[0] for b in bounds]

def getBounds(positions, horizon_threshold, height_threshold, degree_threshold, delta_x):
  position_threshold = (max(positions) - min(positions))*0.01
  bounds = []
  initial_index = -1
  for i in range(len(positions)-1):
    if positions[i+1] > positions[i]:
      if initial_index == -1:
        initial_index = i
    else:
      if initial_index != -1:
        if i - initial_index < horizon_threshold or positions[i] - positions[initial_index] < height_threshold:
          bounds.append((initial_index, i, "white"))
        else:
          bounds.append((initial_index, i, "red"))
      initial_index = -1
  if initial_index != -1:
    if len(positions)-1 - initial_index < horizon_threshold:
      bounds.append((initial_index, len(positions)-1, "white"))
    else:
      bounds.append((initial_index, len(positions)-1, "red"))
  initial_index = -1
  for i in range(len(positions)-1):
    if positions[i+1] < positions[i]:
      if initial_index == -1:
        initial_index = i
    else:
      if initial_index != -1:
        if i - initial_index < horizon_threshold or positions[initial_index] - positions[i] < height_threshold:
          bounds.append((initial_index, i, "white"))
        else:
          bounds.append((initial_index, i, "blue"))
      initial_index = -1
  if initial_index != -1:
    if len(positions)-1 - initial_index < horizon_threshold:
      bounds.append((initial_index, len(positions)-1, "white"))
    else:
      bounds.append((initial_index, len(positions)-1, "blue"))
  bounds.sort(key=lambda x:x[0])
  if len(bounds) == 1: return bounds
  for i, b in enumerate(bounds):
    if b[2] == "white":
      continue
    delta_y = positions[b[0] + delta_x] - positions[b[0]] if b[2] == "red" else positions[b[0]] - positions[b[0] + delta_x]
    alpha = math.degrees(math.atan(0.01 * delta_x / delta_y))
    if (alpha > 90 - degree_threshold) or (-position_threshold <= positions[b[1]]-positions[b[0]] <= position_threshold):
        bounds[i] = (b[0], b[1], "white")
        continue
    if alpha < 46:
        bounds[i] = (b[0], b[1], "green")
  for i, b in enumerate(bounds):
    if i == 0:
      j = 0
      while bounds[j][1] - bounds[j][0] < horizon_threshold:
        j += 1
      bounds[i] = (b[0], b[1], bounds[j][2])
    elif b[1] - b[0] < horizon_threshold:
      bounds[i] = (b[0], b[1], bounds[i-1][2])
  i = 0
  while i < len(bounds)-1:
    if bounds[i][2] == bounds[i+1][2]:
      bounds[i] = (bounds[i][0], bounds[i+1][1], bounds[i][2])
      bounds.pop(i+1)
      i = -1
    i += 1
  filtered_bounds = []
  for b in bounds:
    if b[2] == "green":
      filtered_bounds.append((b[0], int(0.5 * (b[0] + b[1])), "green"))
      filtered_bounds.append((int(0.5 * (b[0] + b[1])), b[1], "blue"))
    else:
      filtered_bounds.append((b[0], b[1], b[2]))
  return filtered_bounds

if __name__ == '__main__':
  print(getPivots())

