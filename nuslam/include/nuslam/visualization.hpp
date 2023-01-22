#ifndef __VISUALIZATION_HPP
#define __VISUALIZATION_HPP

#include <string>
#include <vector>
#include <ros/ros.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nuslam/SlamMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math_utils/math.hpp>

constexpr auto gazebo_world_props_service = "/gazebo/get_world_properties";

namespace colors
{
  inline constexpr double MAGENTA[] = {0.988235294, 0.058823529,
                                       0.752941176, 1};
  inline constexpr double YELLOW[] = {1, 1, 0, 0.9};
  inline constexpr double GREEN[] = {0, 1, 0, 0.5};
}

class Visualization
{
public:
  static int nav_msgs_buffer_size;
  Visualization()
  {
    init();

    ros::service::waitForService(gazebo_world_props_service);
    _landmarks_pub_scheduler = _nh.createTimer(ros::Duration(10),
        &Visualization::publishRealMap, this);
    
    _gazebo_model_states_sub = _nh.subscribe("/gazebo/model_states", 1, 
        &Visualization::publishRealPath, this);
    _real_path_pub = _nh.advertise<nav_msgs::Path>(
        "visualization/real/path", 1);
    _visualization_marker_array_pub = _nh.advertise<
      visualization_msgs::MarkerArray> ("visualization_marker_array", 5);
    _real_path.poses.reserve(nav_msgs_buffer_size);
    _real_path.header.frame_id = "world";
    
    _odom_sub = _nh.subscribe("odom", 10, 
        &Visualization::publishOdomPath, this);
    _odom_path_pub = _nh.advertise<nav_msgs::Path>(
      "visualization/odom/path", 1);
    _odom_path.poses.reserve(nav_msgs_buffer_size);
    _odom_path.header.frame_id = _map_frame;

    _slam_path_pub = _nh.advertise<nav_msgs::Path>(
      "visualization/nuslam/path", 1);
    _slam_path.poses.reserve(nav_msgs_buffer_size);
    _slam_path.header.frame_id = _map_frame;
    _slam_pose_sub = _nh.subscribe("slam/pose", 1, 
        &Visualization::publishSlamPath, this);
    
    _slam_map_sub = _nh.subscribe("slam/map", 1,
        &Visualization::publishSlamMap, this);
  }

  static visualization_msgs::Marker makeMarker(int id, 
    const std::array<double, 2>& pos, const double* color, 
    const std::string& ns, double lifetime, const std::string& map_frame,
    std::array<double, 2> const& scale={0.16, 0.16});

  static visualization_msgs::Marker makeMarker(int id, 
    nuslam::SlamMap::ConstPtr const& msg, const double* color, 
    const std::string& ns, double lifetime, const std::string& map_frame);

  static std::array<double, 2> getCovarianceScale(math::matd<2> const&);

private:
  visualization_msgs::MarkerArray _real_landmarks;
  ros::Publisher _visualization_marker_array_pub;
  ros::Timer _landmarks_pub_scheduler;
  
  ros::Subscriber _gazebo_model_states_sub;
  ros::Subscriber _odom_sub;
  ros::Publisher _real_path_pub;
  ros::Publisher _slam_path_pub;
  ros::Publisher _odom_path_pub;
  ros::Subscriber _slam_pose_sub;
  ros::Subscriber _slam_map_sub;
  nav_msgs::Path _real_path;
  nav_msgs::Path _slam_path;
  nav_msgs::Path _odom_path;

  ros::NodeHandle _nh;
  std::string _robot_name;
  std::string _map_frame;
  std::string _world_frame;

  void init();
  void publishRealMap(ros::TimerEvent const& t);
  void publishSlamMap(nuslam::SlamMap::ConstPtr const& msg);
  void publishRealPath(gazebo_msgs::ModelStates::ConstPtr const& msg);
  void publishOdomPath(nav_msgs::Odometry::ConstPtr const& msg);
  void publishSlamPath(
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr const& slam_msg);
  int getRobotModelId(std::vector<std::string> const& model_names);
  void addPoseToPath(std::vector<geometry_msgs::PoseStamped>& path,
      geometry_msgs::Pose const& pose, std::string const& frame) const;
  
};

int Visualization::nav_msgs_buffer_size = 1000;

void addCylinder(std::string& cylinder_name,
                 std::vector<visualization_msgs::Marker>& cylinders,
                 ros::ServiceClient& get_model,
                 std::string const& map_frame)
{
  gazebo_msgs::GetModelState model_state_srv;
  model_state_srv.request.model_name = cylinder_name;
  if(!get_model.call(model_state_srv))
  {
    ROS_ERROR("Failed to call /gazebo/get_model_state with model_name = %s",
        cylinder_name.c_str());
    return;
  }

  if(model_state_srv.response.success == true)
  {
    auto& res = model_state_srv.response;
    auto& p = res.pose.position;
    auto cylinder = Visualization::makeMarker(cylinders.size(), {p.x, p.y}, 
      colors::GREEN, "visualization/real/map", 0, map_frame);
    cylinders.push_back(cylinder);
  }
}

#endif
