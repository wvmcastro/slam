#!/usr/bin/env python3
from turtle import Turtle
import rospy
from sensor_msgs.msg import LaserScan
from nuslam.msg import LandmarkDebug
from nuslam.msg import TurtleMap
import pickle
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos

class DebugViewer:
  def __init__(self, fig, axs):
    self._fig, self._axs = fig, axs

    self._laser_topic_sub = rospy.Subscriber("scan", 
                                             LaserScan, 
                                             self.laser_scan_callbak)

    self._debug_sub = rospy.Subscriber("nuslam/debug", 
                                       LandmarkDebug,
                                       self.debug_callback)
                                      
    self._landmarks_sub = rospy.Subscriber("slam/measurements",
                                           TurtleMap,
                                           self.save_landmarks)
    
    self._laser_data = None
    self._theta = None

    self._r_theta_plot = None
    self._derivative_plot = None
    self._peaks_plot = None

  def save_landmarks(self, landmarks_data: TurtleMap):
    with open("/home/wellington/landmarks.pickle", "wb+") as output:
      pickle.dump(landmarks_data, output)

  def laser_scan_callbak(self, laser_data: LaserScan):
    self._laser_data = laser_data
    with open("/home/wellington/laser_data.pickle", "wb+") as output:
      pickle.dump(laser_data, output)

    if self._theta is None:
      self._theta = np.linspace(laser_data.angle_min, laser_data.angle_max,
          len(laser_data.ranges))

  def debug_callback(self, debug_data: LandmarkDebug):
    
    with open("/home/wellington/debug_data.pickle", "wb+") as output:
      pickle.dump(debug_data, output)
    
    # r theta plot
    if self._r_theta_plot is None and self._laser_data is not None:
      self._r_theta_plot = self._axs[0].plot(self._theta, 
          self._laser_data.ranges, "b-")[0]
    else:
      self._r_theta_plot.set_ydata(self._laser_data.ranges)
    
    # derivative plot
    if self._derivative_plot is None:
      self._derivative_plot = self._axs[1].plot(self._theta, 
                                                debug_data.signal_derivative,
                                                "g-")[0]
    else:
      self._derivative_plot.set_ydata(debug_data.signal_derivative)
    
    # peaks plot
    index = list(debug_data.peaks_index)
    signal_derivative = np.array(debug_data.signal_derivative)
    if self._peaks_plot is None:
      self._peaks_plot = self._axs[1].plot(self._theta[index],
          signal_derivative[index], "ro")[0]
    else:
      self._peaks_plot.set_xdata(self._theta[index])
      self._peaks_plot.set_ydata(signal_derivative[index])


if __name__ == "__main__":
  rospy.init_node("landmarks_debug", anonymous=True)  
  
  # Use the seborn style
  plt.style.use('seaborn-colorblind')
  # But with fonts from the document body
  plt.rcParams.update({
    "font.family": "serif",  # use serif/main font for text elements
    "text.usetex": True,     # use inline math for ticks
    "pgf.rcfonts": False     # don't setup fonts from rc parameters
  })

  plt.ion()
  fig, axs = plt.subplots(2, 1, constrained_layout=True)
  
  plotter = DebugViewer(fig, axs)
  r = rospy.Rate(30)
  while not rospy.is_shutdown():
    r.sleep()
    fig.canvas.draw()
    fig.canvas.flush_events()

  rospy.spin()
  plt.ioff()
