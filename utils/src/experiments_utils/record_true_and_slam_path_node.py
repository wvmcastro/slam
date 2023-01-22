import sys
import rospy
import tf2_ros
from experiments_utils.record_true_and_slam_path import RobotPoseMonitor, DiskDataWriter

if __name__ == "__main__":
  rospy.init_node("paths_recorder", anonymous=True)
  
  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)
  
  robot_name = sys.argv[1]
  output_folder = sys.argv[2]
  
  monitor = RobotPoseMonitor(robot_name, tfBuffer)

  data_writer = DiskDataWriter(monitor, output_folder)
  rospy.on_shutdown(data_writer)

  rospy.spin()