#include <ros/ros.h>
#include <nuslam/ekf.hpp>
#include <nuslam/slam.hpp>

#include <math_utils/math.hpp>
using math::vec;
using math::mat;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ekf_slam");
  ros::NodeHandle nh;
  
  std::string robot_name{argv[1]};
  
  float wheel_base, wheel_radius;
  nh.getParam("wheel_base", wheel_base);
  nh.getParam("wheel_radius", wheel_radius);
  int map_size;
  nh.getParam("map_size", map_size);
  
  vec<double, 3> x0(0, 0, 0);
  mat<double, 3, 3> cov0 = mat<double, 3, 3>::Zero();
 
  Eigen::DiagonalMatrix<double, 3> process_noise(1e-3, 1e-3, 1e-3); 
  
  double sigma;
  nh.getParam("laser_stddev", sigma);
  Eigen::DiagonalMatrix<double, 2> measurement_noise(sigma, sigma); 

  nuslam::NuslamSystem system{x0, wheel_base, wheel_radius};
  nuslam::EKFSlamDiffDrive ekf(system,
                               x0, cov0, 
                               process_noise, measurement_noise,
                               12);
  
  bool debug;
  ros::NodeHandle _nh("~");
  _nh.getParam("debug", debug);
  auto slam = Slam(ekf, robot_name);
  slam.run();
  ros::spin();
}
