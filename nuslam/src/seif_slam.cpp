#include <ros/ros.h>
#include <nuslam/seif.hpp>
#include <nuslam/slam.hpp>

#include <math_utils/math.hpp>
using math::vecd;
using math::matd;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "seif_slam");
  ros::NodeHandle nh;

  std::string robot_name{argv[1]};

  std::string known_initial_pose{argv[2]};
  std::transform(known_initial_pose.begin(), known_initial_pose.end(), 
    known_initial_pose.begin(), [](unsigned char c){ return std::tolower(c); });

  float wheel_base, wheel_radius;
  nh.getParam("wheel_base", wheel_base);
  nh.getParam("wheel_radius", wheel_radius);
  int map_size;
  nh.getParam("map_size", map_size);
  
  vecd<3> x0(0, 0, 0);
  constexpr double info = 1.0 / 1e-12;
  Eigen::DiagonalMatrix<double, 3> info0(info, info, info);
  Eigen::DiagonalMatrix<double, 3> process_noise(1e-3, 1e-3, 1e-3); 
  
  double sigma;
  nh.getParam("laser_stddev", sigma);
  Eigen::DiagonalMatrix<double, 2> measurement_noise(sigma, sigma); 

  int active_set_size;
  {
    ros::NodeHandle _nh("~");
    _nh.getParam("active_landmarks_set_size", active_set_size);
  }
  ROS_WARN_STREAM("active landmarks set size: " << (uint16_t) active_set_size);
  
  nuslam::NuslamSystem system{x0, wheel_base, wheel_radius};
  nuslam::SEIFSlamDiffDrive seif(system,
                               x0, info0, 
                               process_noise, measurement_noise,
                               map_size,
                               (uint16_t) active_set_size);
                               
  
  auto slam = Slam(seif, robot_name, known_initial_pose == "true");
  slam.run();
  ros::spin();
}
