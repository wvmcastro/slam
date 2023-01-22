#ifndef __MAP_BUILDER_AND_PUBLISHER_HPP
#define __MAP_BUILDER_AND_PUBLISHER_HPP

#include <occupancy_grid_map/OccupancyGrid.hpp>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <math_utils/math.hpp>
#include <occupancy_grid_map/GridMapExchangeSrv.h>
#include <limits>
#include <functional>
#include <string>

class MapBuilderAndPublisher
{
public:
  MapBuilderAndPublisher(const std::string& map_frame, 
                         OccupancyGrid& grid, 
                         tf2_ros::Buffer const& buffer,
                         ros::NodeHandle& nh)
    : _map_frame{map_frame}
    , _grid{grid}
    , _tf2_buffer{buffer}
    , _nh{nh}
  {
    init(nh);
  }

  void publishMap(const sensor_msgs::LaserScan::ConstPtr& lidar_msg);

  void exchangeMap(
    const geometry_msgs::TransformStamped::ConstPtr& exchange_msg);

private:
  std::string _map_frame;
  OccupancyGrid _grid;
  tf2_ros::Buffer const& _tf2_buffer;
  ros::Publisher _pub;
  ros::ServiceServer _exchange_grid_server;
  ros::NodeHandle& _nh;
  std::string _robot_name;

  // params
  float _laser_update_frequency;
  int _scan_sampling_step;


  void init(ros::NodeHandle& nh);
  void getParams(ros::NodeHandle& nh);

  std::vector<std::pair<math::vecd<2>, bool>> 
  getPointsInMapFrame(sensor_msgs::LaserScan const& lidar_msg,
    math::vecd<2> const& sensor_origin_in_map_frame) const;
  
  std::string getRobotName(std::string const& map_frame) const;
  
  void doGridMapIntegration( nav_msgs::OccupancyGrid const& other_grid_msg,
    math::MyTransformData const& transform);
  
  bool handleGridMapExchangeRequest(
    occupancy_grid_map::GridMapExchangeSrv::Request&,
    occupancy_grid_map::GridMapExchangeSrv::Response&);
};

#endif