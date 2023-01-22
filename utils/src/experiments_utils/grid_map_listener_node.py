import rospy
from experiments_utils.grid_map_listener import AreaCoverageMonitor, DiskDataWriter
import sys

if __name__ == "__main__":
  rospy.init_node("OccupancyGridListener", anonymous=True)
  
  robot_name = sys.argv[1]
  output_folder = sys.argv[2]

  monitor = AreaCoverageMonitor(robot_name)
  data_writer = DiskDataWriter(monitor, output_folder)

  rospy.on_shutdown(data_writer)

  rospy.spin()