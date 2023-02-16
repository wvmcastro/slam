import matplotlib.pyplot as plt
import os
import tf

from simulation_utils.utils import get_landmarks_positions

class FullPath:
  def __init__(self, parent):
    self._parent = parent
  
  def get(self, filename):
    return os.path.join(self._parent, filename)

def draw_environment(ax, legend=True):
  ax.set_aspect(1.0)
  env_color = 'k'
  plt.plot([-5,5], [-5,-5], color=env_color, linewidth=3)
  plt.plot([-5,5], [5,5], color=env_color, linewidth=3)
  plt.plot([-5,-5], [-5,5], color=env_color, linewidth=3)
  if legend:
    plt.plot([5,5], [-5,5], color=env_color, linewidth=3, label='paredes')
  else:
    plt.plot([5,5], [-5,5], color=env_color, linewidth=3)

  landmarks = get_landmarks_positions()

  if legend:
    plt.scatter(landmarks[:,0], landmarks[:, 1], s=50, marker='*',\
      color=env_color, label='landmarks')
  else:
    plt.scatter(landmarks[:,0], landmarks[:, 1], s=50, marker='*',\
      color=env_color)

  plt.xlabel("Posição (m)")
  plt.ylabel("Posição (m)")

def Folder(path: str) -> str:
  if not os.path.exists(path):
    raise ValueError
  
  if not os.path.isdir(path):
    raise TypeError
  
  return path

def results_gen(results_folder: Folder):
  for path in os.listdir(results_folder):
    full_path = os.path.join(results_folder, path)
    try:
      yield Folder(full_path)
    except:
      print(f"{full_path} is not a folder")

def toQuaternion(orientation):
  return (orientation.x, orientation.y, orientation.z, orientation.w)

def quaternionToYaw(ros_orientation):
  quat = toQuaternion(ros_orientation)
  euler = tf.transformations.euler_from_quaternion(quat)
  yaw = euler[2]

  return yaw

def is_number(x):
  if type(x) == int:
    return True
  
  if type(x) == float:
    return True
  
  return False