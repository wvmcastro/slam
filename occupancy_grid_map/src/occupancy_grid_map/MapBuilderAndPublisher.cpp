#include <occupancy_grid_map/MapBuilderAndPublisher.hpp>
#include <misc_utils/misc.hpp>

void 
MapBuilderAndPublisher::publishMap(
  const sensor_msgs::LaserScan::ConstPtr& lidar_msg)
{
  std::string error;
  if(!_tf2_buffer.canTransform(_map_frame, 
                               lidar_msg->header.frame_id, 
                               lidar_msg->header.stamp, 
                               ros::Duration(1.0 / _laser_update_frequency), // wait while the slam node process updated map transform
                               &error))
  {
    ROS_ERROR_STREAM("error: " << error);
    return ;
  }
  
  geometry_msgs::PointStamped origin;
  origin.header = lidar_msg->header;
  origin = _tf2_buffer.transform(origin, _map_frame);
  math::vecd<2> sensor_origin_in_map_frame{origin.point.x, origin.point.y};

  auto points = getPointsInMapFrame(*lidar_msg, sensor_origin_in_map_frame);

  if(points.size() == 0)
    return;

  for(auto& point : points)
    _grid.updateMap(sensor_origin_in_map_frame, point.first, point.second);

  auto msg = _grid.gridMapMessage(_map_frame, lidar_msg->header.stamp);
  _pub.publish(msg);
}

void
MapBuilderAndPublisher::init(ros::NodeHandle& nh)
{
  constexpr char map_topic[] = "map";
  _pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);

  _robot_name = getRobotName(_map_frame);

  getParams(nh);

  _exchange_grid_server = nh.advertiseService("grid_map_exchange",
    &MapBuilderAndPublisher::handleGridMapExchangeRequest, 
    this);
}

void
MapBuilderAndPublisher::getParams(ros::NodeHandle& nh)
{
  nh.getParam("laser_update_rate", _laser_update_frequency);
  ros::param::get("~scan_sampling_step", _scan_sampling_step);
}

std::vector<std::pair<math::vecd<2>, bool>> 
MapBuilderAndPublisher::getPointsInMapFrame(
  sensor_msgs::LaserScan const& lidar_msg,
  math::vecd<2> const& sensor_origin_in_map_frame) const
{
  math::MyTransformData transform_lidar_to_map = misc::getTransform(_map_frame,
    lidar_msg.header.frame_id, _tf2_buffer, lidar_msg.header.stamp);

  const auto& ranges = lidar_msg.ranges;
  int n = ranges.size();
  std::vector<std::pair<math::vecd<2>, bool>> buffer;
  buffer.reserve(n);

  float theta0 = transform_lidar_to_map.rotation + lidar_msg.angle_min;
  for(int i = 0; i < n; i+=_scan_sampling_step)
  {
    if(ranges[i] < lidar_msg.range_min)
      continue;

    double p[] = {ranges[i], theta0 + i*lidar_msg.angle_increment};
    
    bool free = false;
    if(p[0] == std::numeric_limits<float>::infinity())
    {
      p[0] = lidar_msg.range_max;
      free = true;
    }
    
    math::polarToCartesian(p);
    buffer.emplace_back(sensor_origin_in_map_frame + math::vecd<2>(p), free);
  }

  return buffer;
}


void
MapBuilderAndPublisher::exchangeMap(
  const geometry_msgs::TransformStamped::ConstPtr& exchange_msg)
{
  if(getRobotName(exchange_msg->child_frame_id) != _robot_name) 
    return;

  std::string other_robot_name = getRobotName(exchange_msg->header.frame_id);
  
  ROS_INFO_STREAM(_robot_name << " requests grid map to " << other_robot_name);

  std::string exchange_service = "/" + other_robot_name + "/grid_map_exchange";
  ros::ServiceClient client =
    _nh.serviceClient<occupancy_grid_map::GridMapExchangeSrv>(
      exchange_service);

  occupancy_grid_map::GridMapExchangeSrv srv;
  srv.request.robot_id = _robot_name;
  if(client.call(srv) == false)
  {
    ROS_ERROR_STREAM("Grid map request to " << exchange_service << " FAILED");
    return;
  }
  
  ROS_INFO_STREAM("Grid map data received from " << other_robot_name);

  tf2::Quaternion quat;
  tf2::fromMsg(exchange_msg->transform.rotation, quat);
  double rotation = math::quaternionToHeading<double>(quat);
  if(std::abs(rotation) > 5e-2)
  {
    ROS_ERROR_STREAM("Can't handle grid rotation right now. Exchange aborted!");
    return;
  }

  math::MyTransformData tr(exchange_msg->transform.translation.x,
    exchange_msg->transform.translation.y, rotation);
  doGridMapIntegration(srv.response.map, tr);
}


void
MapBuilderAndPublisher::doGridMapIntegration(
  nav_msgs::OccupancyGrid const& other_grid_msg,
  math::MyTransformData const& transform)
{
  ROS_WARN_STREAM("Merge occupancy grid maps");
  OccupancyGrid other_map(other_grid_msg);
  _grid.merge(other_map, transform);
}

bool
MapBuilderAndPublisher::handleGridMapExchangeRequest(
  occupancy_grid_map::GridMapExchangeSrv::Request& req,
  occupancy_grid_map::GridMapExchangeSrv::Response& response)
{
  ROS_INFO_STREAM("Sending grid map data to " << req.robot_id);
  response.map = _grid.gridMapMessage(_map_frame, ros::Time::now());
  return true;    
}


std::string
MapBuilderAndPublisher::getRobotName(std::string const& map_frame) const
{
  int slash_pos = map_frame.find("/");
  if(slash_pos == std::string::npos)
    return std::string();

  return std::string(map_frame.data(), slash_pos);
}