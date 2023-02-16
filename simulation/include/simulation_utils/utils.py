import rospkg
import os
import numpy as np
from xml.dom import minidom

def get_world_file():
  rospack = rospkg.RosPack()

  nunavigation_path = rospack.get_path("nunavigation")
  world_path = os.path.join(nunavigation_path, "worlds", "big-block.world")

  return world_path

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