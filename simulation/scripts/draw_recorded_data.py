#!/usr/bin/env python3

from argparse import ArgumentParser
import os
import pickle
from itertools import zip_longest
import matplotlib.pyplot as plt
import numpy as np

from utils import FullPath, Folder, results_gen, draw_environment, is_number

def get_parser() -> ArgumentParser:
  parser = ArgumentParser()

  parser.add_argument("results_folder", type=Folder, help="folder \
    with the simulations")
  
  return parser

def is_valid_position(position):
  return is_number(position.x) and is_number(position.y)

def get_positions_from_pose_file(file: str):
  with open(file, "rb") as f:
    poses = pickle.load(f)

  true_positions, slam_positions = [], [] 
  for true, slam in zip_longest(poses.true, poses.slam, fillvalue=None):
    if true is not None and is_valid_position(true.position):
      true_positions.append((true.position.x, true.position.y))
    if slam is not None and is_valid_position(slam.position):
      slam_positions.append((slam.position.x, slam.position.y))
  
  return true_positions, slam_positions

def plot_path(robot_path_data:dict, robot_name:str = "", legend=True):
  if len(robot_path_data["true_position"]) == 0:
    return
  
  true_path = np.array(robot_path_data["true_position"])
  if legend == True: 
    plt.scatter(true_path[0,0], true_path[0,1], marker='X', c='magenta', s=50, label='início')
    plt.scatter(true_path[-1,0], true_path[-1,1], marker='X', c='maroon', s=50, label='fim')
  else:
    plt.scatter(true_path[0,0], true_path[0,1], marker='X', c='magenta', s=50)
    plt.scatter(true_path[-1,0], true_path[-1,1], marker='X', c='maroon', s=50)
  
  plt.plot(true_path[:,0], true_path[:,1], linewidth=1, label=f"real {robot_name}")
  
  slam_path = np.array(robot_path_data["slam_position"])
  plt.plot(slam_path[:,0], slam_path[:,1], linewidth=1, label=f"slam {robot_name}")

  plt.title("Comparação caminho real com o caminho estimado")
  
  plt.legend(bbox_to_anchor=(0.1, 0.8), loc='center right')


def draw_path(result_folder: Folder):
  full_path = FullPath(result_folder)

  poses_files = [] 
  for file in os.listdir(result_folder):
    if "poses" in file:
      poses_files.append(file)

  position_data = {}
  for file in poses_files:
    true_position, slam_position = get_positions_from_pose_file(full_path.get(file))
    robot_name = file.split('-')[-1].split('.')[0]
    position_data[robot_name] = {
      'true_position': true_position,
      'slam_position': slam_position
    }
  
  for robot, robot_data in position_data.items():
    fig, ax = plt.subplots()
    draw_environment(ax, legend=False)
    plot_path(robot_data)
    plt.savefig(full_path.get(f"{robot}-path.pdf"))
    plt.close()
  
  if len(position_data) > 1:
    fig, ax = plt.subplots()
    draw_environment(ax, legend=False)
    for i, (robot, robot_data) in enumerate(position_data.items()):
      plot_path(robot_data, f"Robô {i+1}", True if i == 0 else False)
    
    plt.savefig(full_path.get(f"robots-path.pdf"))
    plt.close()


if __name__ == "__main__":
  parser = get_parser()
  args = parser.parse_args()

  full_path = FullPath(args.results_folder)

  results = results_gen(args.results_folder)
  for result in results:
    draw_path(full_path.get(result))