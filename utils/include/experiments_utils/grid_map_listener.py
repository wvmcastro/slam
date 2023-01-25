#!/usr/bin/env python3
import rospy
import time
import pickle
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
import sys
from bisect import bisect_right

def get_area(msg: OccupancyGrid) -> float:
  res = msg.info.resolution
  cell_area = res*res

  data = np.array(msg.data)
  n = sum(data != -1)

  return n*cell_area

class State:
  def __init__(self, robot_name=""):
    self.robot_name = robot_name
    self.data = ([], [])
    self.occupancy_grid = None

class AreaCoverageMonitor:
  def __init__(self, robot_name: str):
    self.robot_name = robot_name
    self.state = State(robot_name)

    self.sub = rospy.Subscriber(f"{robot_name}/map", OccupancyGrid, 
      self.occupancy_grid_callback)

  def occupancy_grid_callback(self, msg: OccupancyGrid):
    self.state.occupancy_grid = msg
    
    t, area = self.state.data
    t.append(rospy.Time.now().to_sec())
    area.append(get_area(msg))

  def covered_area(self) -> float:
    if len(self.state.data[1]) == 0:
      return 0

    return self.state.data[1][-1]
  
  def area_coverage_time(self, area):
    index = bisect_right(self.state.data[1], area)
    return self.state.data[0][index] - self.state.data[0][0]
  
class DiskDataWriter:
  def __init__(self, monitor: AreaCoverageMonitor, root: str):
    self.monitor = monitor
    self.root = root

  def __call__(self) -> None:
    self.save_data_on_disk()
  
  def save_data_on_disk(self):
    robot_name = self.monitor.robot_name

    with open(os.path.join(self.root, f"area-coverage-{robot_name}.pickle"), "wb+") as output:
      pickle.dump(self.monitor.state.data, output)
    
    with open(os.path.join(self.root, f"occupancy-grid-{robot_name}.pickle"), "wb+") as output:
      pickle.dump(self.monitor.state.occupancy_grid, output)
