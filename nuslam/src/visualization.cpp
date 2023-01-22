#include "nuslam/visualization.hpp"
#include <math_utils/math.hpp>

void
Visualization::init()
{
  _nh.getParam("robot_name", _robot_name);
  _map_frame = _robot_name + "/map";
  _world_frame = "world";
}

void 
Visualization::publishRealMap(ros::TimerEvent const& t)
{
  if(_real_landmarks.markers.size() == 0)
  {
    auto gazebo_world_props_client = _nh.serviceClient<
      gazebo_msgs::GetWorldProperties>(gazebo_world_props_service);
    
    gazebo_msgs::GetWorldProperties world_props_srv;
    if(!gazebo_world_props_client.call(world_props_srv))
    {
      ROS_ERROR("Failed to call service %s", gazebo_world_props_service);
      return;
    }

    auto& res = world_props_srv.response;
    if(res.success == true)
    {
      auto get_model = _nh.serviceClient<gazebo_msgs::GetModelState>(
          "/gazebo/get_model_state");
      for(std::string& model : res.model_names)
      {
        if(model.substr(0, 8) == "cylinder")
          addCylinder(model, _real_landmarks.markers, get_model, _world_frame);
      }
      _visualization_marker_array_pub.publish(_real_landmarks);
    }
  }
  else
    _visualization_marker_array_pub.publish(_real_landmarks);
}

void
Visualization::publishSlamMap(nuslam::SlamMap::ConstPtr const& msg)
{
  static visualization_msgs::MarkerArray _slam_map;
  static int last_size = 0;

  int n = msg->x.size();
  for(int i = 0; i < last_size; i++)
  {
    _slam_map.markers[i].pose.position.x = msg->x[i];
    _slam_map.markers[i].pose.position.y = msg->y[i];
  }

  for(int i = last_size; i < n; i++)
  {
    _slam_map.markers.emplace_back(makeMarker(i, msg,
      colors::YELLOW, "visualization/slam/map", 0, _map_frame));
  }

  _visualization_marker_array_pub.publish(_slam_map);
  last_size = n;
}

visualization_msgs::Marker 
Visualization::makeMarker(int id, const std::array<double, 2>& pos, 
  const double* color, const std::string& ns, double lifetime, 
  std::string const& map_frame, std::array<double, 2> const& scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame;
  marker.ns = ns;
  marker.id = id; 
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(lifetime); 
  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.orientation.w = 1;
  marker.scale.x = scale[0];
  marker.scale.y = scale[1];
  marker.scale.z = 0.4;
  marker.pose.position.z = marker.scale.z / 2.0;
  marker.color.r = color[0]; 
  marker.color.g = color[1]; 
  marker.color.b = color[2]; 
  marker.color.a = color[3]; 
  return marker;
}

visualization_msgs::Marker 
Visualization::makeMarker(int id, nuslam::SlamMap::ConstPtr const& msg, 
  const double* color, const std::string& ns, double lifetime, 
  std::string const& map_frame)
{
  return makeMarker(id, {msg->x[id], msg->y[id]}, color, ns, lifetime,
    map_frame, {0.16, 0.16});
}

void
Visualization::publishRealPath(gazebo_msgs::ModelStates::ConstPtr const& msg)
{
  static int robot_model_id = getRobotModelId(msg->name);
  static ros::Time last = ros::Time::now(); 
  if(robot_model_id != -1)
  {
    auto now = ros::Time::now();
    if((now-last).toSec() >= 1/30.0)
    {
      auto& robot_pose = msg->pose[robot_model_id];
      addPoseToPath(_real_path.poses, robot_pose, "world");
      _real_path.header.stamp = now;
      _real_path_pub.publish(_real_path);
      last = now;
    }
  }
  else
  {
    robot_model_id = getRobotModelId(msg->name);
  }
}

void 
Visualization::publishOdomPath(nav_msgs::Odometry::ConstPtr const& msg)
{
  static ros::Time last = ros::Time::now();
  
  auto now = ros::Time::now();
  if((now-last).toSec() >= 1/30.0)
  {
    _odom_path.header.stamp = now;
    auto& robot_pose = msg->pose.pose;
    addPoseToPath(_odom_path.poses, robot_pose, _map_frame);
    _odom_path_pub.publish(_odom_path);
    last = now;
  }
}

void 
Visualization::publishSlamPath(
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr const& msg)
{
  static ros::Time last = ros::Time::now();
  
  auto now = ros::Time::now();
  if((now-last).toSec() >= 1/30.0)
  {
    _slam_path.header.stamp = now;
    auto& robot_pose = msg->pose.pose;
    addPoseToPath(_slam_path.poses, robot_pose, _map_frame);
    _slam_path_pub.publish(_slam_path);
    last = now; 
  }
}

int
Visualization::getRobotModelId(std::vector<std::string> const& model_names)
{
  for(int i = 0; i < model_names.size(); i++)
    if(model_names[i] == _robot_name)
      return i;
  return -1;
}

void 
Visualization::addPoseToPath(std::vector<geometry_msgs::PoseStamped>& path,
      geometry_msgs::Pose const& pose, std::string const& frame) const
{
  geometry_msgs::PoseStamped new_pose;
  new_pose.header.stamp = ros::Time::now();
  new_pose.header.frame_id = frame;
  new_pose.pose = pose;

  if(path.size() < Visualization::nav_msgs_buffer_size)
    path.push_back(new_pose);
  else
  {
    int last_index = Visualization::nav_msgs_buffer_size - 1;
    for(int i = 0; i < last_index; i++)
      path[i] = path[i+1];

    path[last_index] = new_pose;
  }
}

std::array<double, 2> 
Visualization::getCovarianceScale(math::matd<2> const& cov)
{
  Eigen::EigenSolver<math::matd<2>> eig(cov);
  auto eigenvalues = eig.eigenvalues().real();

  return std::array<double, 2>{ 2*eigenvalues(0),
    2*eigenvalues(1) };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization");
  
  Visualization visualizer;
  
  ros::spin();
}
