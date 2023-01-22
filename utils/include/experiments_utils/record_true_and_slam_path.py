import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf
import tf2_geometry_msgs
import pickle
import os
import numpy as np
import sys

def pos_diff(pose1, pose2):
  return np.array((
    pose1.position.x - pose2.position.x,
    pose1.position.y - pose2.position.y
  ))  


def get_orientation_diff(orientation_a, orientation_b):
  euler_a = tf.transformations.euler_from_quaternion(orientation_a)
  euler_b = tf.transformations.euler_from_quaternion(orientation_b)

  return euler_a[2] - euler_b[2]

def toQuaternion(orientation):
  return (orientation.x, orientation.y, orientation.z, orientation.w)

def get_orientation_errors(poses_a, poses_b):
  errors = []
  for i in range(len(poses_a)):
    orientation_a = toQuaternion(poses_a[i].orientation)
    orientation_b = toQuaternion(poses_b[i].orientation)

    errors.append(get_orientation_diff(orientation_a, orientation_b))

  return errors

class Poses:
  def __init__(self):
    self.true = []
    self.slam = []
    self.odom = []

class State:
  def __init__(self):
    self.last_true_pose = None
    self.last_odom_pose = None
    self.last_slam_pose = None

class RobotPoseMonitor:
  def __init__(self, robot_name: str, tf_buffer: tf2_ros.Buffer):
    self.robot_name = robot_name
    self.tf_buffer = tf_buffer

    self.poses = Poses()
    self.state = State()

    self.sub_true_pose = rospy.Subscriber("/gazebo/model_states", ModelStates,
      self.listen_real_pose)
    
    self.sub_slam_pose = rospy.Subscriber(robot_name+"/slam/pose",
      PoseWithCovarianceStamped,
      self.listen_slam_pose)
    
    self.sub_odom_pose = rospy.Subscriber(robot_name+"/odom", Odometry,
      self.listen_odom_pose)

  def find_robot_index(self, models_names) -> str:
    for i, name in enumerate(models_names):
      if name == self.robot_name:
        return i
    
    return None

  def listen_real_pose(self, msg: ModelStates):
    robot_index = self.find_robot_index(msg.name)

    if robot_index is None:
      return

    self.state.last_true_pose = msg.pose[robot_index]

  def listen_odom_pose(self, msg: Odometry):
    try:
      trans = self.tf_buffer.lookup_transform("world", self.robot_name+"/map", rospy.Time())
      pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
      self.state.last_odom_pose = pose_transformed.pose
    except Exception as e:
      print(e)

  def listen_slam_pose(self, msg: PoseWithCovarianceStamped):
    try:
      frame = msg.header.frame_id
      trans = self.tf_buffer.lookup_transform("world", frame, rospy.Time(),
        rospy.Duration(1.0/5.0))

      pose_transformed = tf2_geometry_msgs.do_transform_pose(msg.pose, trans)
      self.state.last_slam_pose = pose_transformed.pose
      self.append()
    except Exception as e:
      print(e)
    finally:
      self.state = State()

  def append(self) -> None:
    if self.state.last_odom_pose is None or self.state.last_slam_pose is None \
      or self.state.last_true_pose is None:
      return

    self.poses.true.append(self.state.last_true_pose)
    self.poses.slam.append(self.state.last_slam_pose)
    self.poses.odom.append(self.state.last_odom_pose)
  
  def slam_error(self, index=-1):
    if len(self.poses.slam) == 0:
      return 0

    slam_pose = self.poses.slam[index]
    true_pose = self.poses.true[index]

    err = np.linalg.norm(pos_diff(slam_pose, true_pose))

    return err
  
  def get_position_mean_error(self):
    error = 0
    n = len(self.poses.slam)
    for i in range(n):
      error += self.slam_error(i)
    
    return error / n
  
  def get_heading_mean_error(self):
    errors = get_orientation_errors(self.poses.slam, self.poses.true)
    error = 0
    for err in errors:
      error += abs(err)
    
    return error / len(self.poses.slam)


class DiskDataWriter:
  def __init__(self, monitor: RobotPoseMonitor, output_folder: str):
    self.output_folder = output_folder
    self.monitor = monitor

  def __call__(self):
    self.save_poses()

  def save_poses(self):
    with open(os.path.join(self.output_folder, 
      f"poses-{self.monitor.robot_name}.pickle"), "wb+") as output:
      
      pickle.dump(self.monitor.poses, output)
