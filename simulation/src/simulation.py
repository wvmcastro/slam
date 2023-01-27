#!/usr/bin/env python3

import rospy
import rospkg
import os
import numpy as np
from xml.dom import minidom
import random
from argparse import ArgumentParser
import yaml
from datetime import datetime
import json
from time import sleep, time
from random import seed

import rospy
import tf2_ros

from launch_utils import launch_file_and_args, get_launcher,\
  start_communication_simulator

from communication_simulator import CommunicationSimulator

from experiments_utils.grid_map_listener import AreaCoverageMonitor
from experiments_utils.record_true_and_slam_path import RobotPoseMonitor

def launch_robot(robot_name, x, y, record_path, known_initial_pose=False, map_size=50):
  pkg = "nunavigation"
  launch_file = "robot_seifslam.launch"

  cli_args = [pkg, launch_file, f"map_size:={map_size}",\
    f"robot_name:={robot_name}", f"ox:={x}", f"oy:={y}",\
      f"known_initial_pose:={known_initial_pose}",
      "record_experiment_data:=True", f"record_path:={record_path}"]

  return launch_file_and_args(cli_args)


def get_world_file():
  rospack = rospkg.RosPack()

  nunavigation_path = rospack.get_path("nunavigation")
  world_path = os.path.join(nunavigation_path, "worlds", "big-block.world")

  return world_path

def launch_simulator(paused:str = "True", gui: str="True"):
  world_path = get_world_file()

  pkg = "gazebo_ros"
  launch_file = "empty_world.launch"
  cli_args = [pkg, launch_file, f"world_name:={world_path}", f"paused:={paused}",\
   f"gui:={gui}", "use_sim_time:=True"]
  
  return launch_file_and_args(cli_args)

def get_landmarks_positions():
  world_file = get_world_file()
  world_xml = minidom.parse(world_file)

  positions = []
  models = world_xml.getElementsByTagName('state')[0].getElementsByTagName('model')
  for model in models:
    model_name = model.attributes['name'].value
    if "cylinder" in model_name:
      pose_element = model.getElementsByTagName('pose')[0]
      pose = pose_element.firstChild.nodeValue.split()
      x = float(pose[0])
      y = float(pose[1])
      positions.append((x,y))
  
  return np.array(positions)

def get_min_distance(point:np.ndarray, points_set:np.ndarray):
  diff = points_set - point
  distances = np.linalg.norm(diff, axis=1)
  return distances.min()


def get_robot_initial_random_position(positions):
  range_min = -45
  range_max = 45
  while True:
    x = random.randint(range_min, range_max) / 10.0
    y = random.randint(range_min, range_max) / 10.0
    pos = np.array([x,y])
    min_d = get_min_distance(pos, positions)
    if min_d > 0.8:
      return pos

def launch_rviz(robot_name):
  pkg = "nunavigation"
  launch_file = "rviz.launch"

  cli_args = [pkg, launch_file, f"robot_name:={robot_name}", "algorithm:=seif"]

  return launch_file_and_args(cli_args)

def launch_communication_simulator(range:float, robots):
  pkg="simulation"
  launch_file = "comm_simulator.launch"

  robots_list = '\"'
  for robot in robots:
    robots_list += robot + ' '
  robots_list += '\"'

  robots_list = "\"rb1 rb2 \""
  cli_args = [pkg, launch_file, f"communication_range:={range}",\
    f"robots:={robots_list}"]

  return launch_file_and_args(cli_args)

def launch_robots(robots_configs, info, record_path):
  positions = get_landmarks_positions()

  robots_launch_files = []
  info['robots'] = []
  for robot_config in robots_configs:
    rb_pos = get_robot_initial_random_position(positions)
    positions = np.vstack([positions, rb_pos])
    
    robot = robot_config["name"]
    robots_launch_files.append(launch_robot(robot, *rb_pos, record_path,\
      robot_config["informe_initial_pose"]))

    if robot_config["rviz"] == True:
      robots_launch_files.append(launch_rviz(robot))
    
    info['robots'].append({
      "name": robot,
      "initial_position": list(rb_pos),
      "rviz": False,
      "informe_initial_pose": robot_config["informe_initial_pose"]
    })

  return tuple(robots_launch_files)

def get_args_parser():
  parser = ArgumentParser()
  parser.add_argument("config_file", type=str, help="simulation config file")

  return parser

def prepare_workspace(workspace_root_folder) -> str:
  if os.path.exists(workspace_root_folder) == False:
    os.makedirs(workspace_root_folder)
  
  dt = datetime.now()
  dt_str = dt.strftime("%Y-%m-%dT%H-%M-%S") 
  workspace = os.path.join(workspace_root_folder, dt_str)
  os.makedirs(workspace)

  return workspace

def save_info(workspace, info):
  filename = os.path.join(workspace, 'info.json')
  with open(filename, 'w+') as f:
    json_str = json.dumps(info, indent=2)
    f.write(json_str)

def launch_topics_recorder(workspace,
    topics: str='(.*)slam/measurements|(.*)joint_states'):
  pkg = "simulation"
  launch_file = "recorder.launch"

  record_path = os.path.join(workspace, 'topics')
  cli_args = [pkg, launch_file, f"record_path:={record_path}",\
    f"topics:={topics}"]  

  return launch_file_and_args(cli_args)

class SimulationMonitor:
  def __init__(self, launcher, robots, area_coverage_th):
    self.simulation_launcher = launcher
    self.robots = robots
    self.area_coverage_th = area_coverage_th
    self.pose_monitors = []
    self.area_coverage_monitors = []

    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    self.init_monitors(robots)
    self.t0 = rospy.Time.now()

    self.robot_position_error_th = 1 # one meter
    self.count = 0
    self.count_limit = 20
    self.failure_reason = ""
  
  def init_monitors(self, robots):
    for robot in robots:
      self.area_coverage_monitors.append(AreaCoverageMonitor(robot))
      self.pose_monitors.append(RobotPoseMonitor(robot, self.tfBuffer))
  
  def success(self) -> bool:
    area_covered = True
    for area_monitor in self.area_coverage_monitors:
      area_covered &= (area_monitor.covered_area() >= self.area_coverage_th)
    
    return area_covered
  
  def failure(self) -> bool:
    dt = (rospy.Time.now() - self.t0).to_sec()

    max_simulation_time = 60*17 # 17min
    if dt >= max_simulation_time:
      self.failure_reason = "time limit exceeded"
      return True

    for monitor in self.pose_monitors:
      error = monitor.slam_error(-1)
      if error > self.robot_position_error_th:
        self.failure_reason = "robot position error too large"
        return True

    return False

  def save_statistics(self, info):
    for i, robot in enumerate(self.robots):
      info[robot] = {}
      info[robot]["area_coverage_time"] = self.area_coverage_monitors[i].area_coverage_time(self.area_coverage_th)
      info[robot]["mean_position_iae"] = self.pose_monitors[i].get_position_mean_error()
      info[robot]["mean_heading_iae"] = self.pose_monitors[i].get_heading_mean_error()

  def run(self, info) -> None:
    status = "none"
    while True:
      if self.success():
        status = "success"
        rospy.logdebug("simulation terminated successfully")
        break

      if self.failure():
        status = "failure"
        break
      
      self.simulation_launcher.spin_once()  
    
    
    info["status"] = status
    info["total_time"] = (rospy.Time.now() - self.t0).to_sec()

    if status == "success":
      info["statistics"] = {}
      self.save_statistics(info["statistics"])
    else:
      info["reason"] = self.failure_reason


if __name__ == "__main__":
  seed(time())
  args_parser = get_args_parser()
  args = args_parser.parse_args()
  
  rospy.init_node("simulator")

  with open(args.config_file, "r") as f:
    config = yaml.safe_load(f)
  workspace = prepare_workspace(config['output_folder'])
  
  launch_files = []

  if "record_topics" in config:
    if config["record_topics"]["enabled"] == True:
      topics_recorder_launch = launch_topics_recorder(workspace)
      launch_files.append(topics_recorder_launch)
  
  info = {}
  info["setup"] = {}
  robots_launch = launch_robots(config["robots"], info["setup"], workspace)
  launch_files += robots_launch

  # gazebo_launch = launch_simulator(paused="False", gui="False")

  launcher = get_launcher(launch_files) 


  launcher.start()
  sleep(10)
  
  robots_names = [robot["name"] for robot in config["robots"]] 
  comm_range = config["communication_range"]
  comm_simulator = CommunicationSimulator(robots_names, comm_range)
  info["setup"]["comm_range"] = comm_range
  
  rospy.logdebug("simulation started") 

  info["simulation_result"] = {}
  simulation_monitor = SimulationMonitor(launcher, robots_names, 90)
  simulation_monitor.run(info["simulation_result"])
  
  rospy.logdebug("simulation finished") 
  
  save_info(workspace, info) 

  del simulation_monitor
  launcher.shutdown()

  rospy.signal_shutdown("shutdown")