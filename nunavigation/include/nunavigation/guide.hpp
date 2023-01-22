#ifndef __GUIDE_HPP_
#define __GUIDE_HPP_

#include <ros/ros.h>
#include <math_utils/math.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using map = Eigen::Matrix<signed char, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class Guide
{
public:
  Guide(const std::string& map_topic, const std::string pose_topic)
  {
    init(map_topic, pose_topic);
  }

  void init(const std::string& map_topic, const std::string& pose_topic);
  
  std::pair<math::vecf<2>, std::string> nextGoal();
  constexpr static float padding_size = 0.4; // meters
  constexpr static float occupied_threshold = 60; // percent

private:
  ros::Subscriber _map_sub;
  ros::Subscriber _pose_sub;
  ros::NodeHandle _nh;

  math::vecf<2> _position;
  map _map;
  nav_msgs::MapMetaData _map_metadata;
  std::string _map_frame;


  void mapCallback(nav_msgs::OccupancyGrid::ConstPtr const&);
  void poseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr const&);

  math::veci<2> mapToGrid(math::vecf<2> const& p) const;
  math::vecf<2> gridToMap(math::veci<2> const& p) const;
  
  bool check_free_neighborhood(uint32_t row, uint32_t col, 
    uint32_t neighborhood_padding);

  uint32_t get_neighborhood_padding(double grid_resolution) const;
};

#endif
