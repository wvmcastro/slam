#!/usr/bin/env python3

import numpy as np
import sys
import time
import rospy

from gazebo_msgs.msg import ModelStates
from nuslam.msg import MapExchangeCommand

class State:
  def __init__(self, robots: list, communication_range):
    self.robots = robots
    n = len(robots)
    self.distances = np.full((n,n), np.inf) 

def getRobotsIndexes(robots_names, models_list):
  indexes = []

  s = len(models_list)
  for name in robots_names:
    for k in range(0, s):
      if name == models_list[k]:
        indexes.append(k)
        break
  
  return tuple(indexes)

def getPositions(robots_indexes, poses):
  positions = []

  for robot in robots_indexes:
    x = poses[robot].position.x
    y = poses[robot].position.y
    positions.append(np.array([x, y]))
  
  return positions

def calculateDistances(distance_matrix, positions):
  n = len(positions)
  for i in range(0, n):
    for j in range(i, n):
      d = np.linalg.norm(positions[i] - positions[j])
      distance_matrix[i,j] = d
      distance_matrix[j,i] = d
  


def getCommunications(state: State, communication_range):
  communications = []

  distance_matrix = state.distances

  n = distance_matrix.shape[0]
  for i in range(0, n):
    for j in range(i+1, n):
      if distance_matrix[i,j] < communication_range:
        communications.append((state.robots[i], state.robots[j]))

  return communications


class CommunicationSimulator:
  def __init__(self, robots: list, range: float):
    self.range = range
    self.robots = robots
    self.state = State(robots, range)


    self.models_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates,
      self.modelStatesCallback)
  
    self.comm_pub = rospy.Publisher("/map_exchange_command", MapExchangeCommand, 
      queue_size=1)

    self.last_pub_time = time.time()

  def modelStatesCallback(self, msg: ModelStates):

    dt = time.time() - self.last_pub_time
    if dt >= 1:
      indexes = getRobotsIndexes(self.state.robots, msg.name)
      positions = getPositions(indexes, msg.pose)
      calculateDistances(self.state.distances, positions)
      pairs = getCommunications(self.state, self.range)

      self.last_pub_time = time.time()
      if len(pairs) > 0:
        for pair in pairs:
          command = MapExchangeCommand()
          command.peer1 = pair[0]
          command.peer2 = pair[1]
          self.comm_pub.publish(command)
      else:
        self.comm_pub.publish(MapExchangeCommand())
        

if __name__ == "__main__":
  rospy.init_node("communication_simulator", anonymous=True)
  
  rospy.loginfo("Running")

  communication_range = float(sys.argv[1])
  rospy.logerr(f"communication range {communication_range}") 
  robots_names = sys.argv[2]
  rospy.logerr(f"robot names: {robots_names}")
  
  comm_simulator = CommunicationSimulator(robots_names.split(), communication_range)

  rospy.sleep(5)
  rospy.spin()