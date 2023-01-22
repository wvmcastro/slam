#include "occupancy_grid_map/OccupancyGrid.hpp"
#include <ros/ros.h>
#include <graphics_utils/bresenham.hpp>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void 
OccupancyGrid::updateMap(Eigen::Vector2d const& ray_origin, 
                         Eigen::Vector2d const& ray_end,
                         bool end_point_free)
{
  auto origin = mapToGridCoordinate(ray_origin);
  if(update_map_size(origin) == false)
    return;
  
  auto end = mapToGridCoordinate(ray_end);
  if(update_map_size(end) == false)
    return;

  _map(origin[0], origin[1]) += log_free;

  if(end_point_free == false)
    _map(end[0], end[1]) += log_occupied;

  auto cells = bresenham(origin, end);
  int n = cells.size();
  for(int i = 1; i < n-1; i++)
  {
    auto& c = cells[i];
    _map(c[0], c[1]) += log_free;
  }
}

veci<2> 
OccupancyGrid::mapToGridCoordinate(vecd<2> const& coord) const
{
  auto& mo = _map_origin;
  auto dcoord = coord.array() / resolution + 0.5;
  veci<2> grid_coord{mo[0]+dcoord[1], mo[1]+dcoord[0]};
  return grid_coord;
}

bool 
OccupancyGrid::update_map_size(veci<2> const& cell)
{
  return update_extremes(_bottom_left[0], _top_right[0], cell[0])
    && update_extremes(_bottom_left[1], _top_right[1], cell[1]);
}

nav_msgs::OccupancyGrid 
OccupancyGrid::gridMapMessage(std::string const& frame_id, 
  ros::Time const& stamp) const
{
  nav_msgs::OccupancyGrid msg{};
  msg.header.frame_id = frame_id;
  msg.info.map_load_time = stamp;
  msg.info.resolution =  resolution;

  msg.info.width = width();
  msg.info.height = height();

  auto& bl = _bottom_left;

  geometry_msgs::Pose origin;
  origin.position.x = (bl[1] - _map_origin[1])*resolution - 0.5*resolution;
  origin.position.y = (bl[0] - _map_origin[0])*resolution - 0.5*resolution;
  origin.position.z = 0;
  origin.orientation = tf2::toMsg(math::headingToQuaternion(0));
  msg.info.origin = origin;

  // fill data
  for(int i = bl[0]; i < bl[0]+height(); i++)
    for(int j = bl[1]; j < bl[1]+width(); j++)
    {
      if(_map(i,j) == 0) // unknown
        msg.data.push_back(-1);
      else
      {
        double p = math::log_odds_to_probability(_map(i,j));
        msg.data.push_back( 100 * p );
      }
    }

  return msg;
}

OccupancyGrid::OccupancyGrid(nav_msgs::OccupancyGrid const& msg):
  resolution{msg.info.resolution}
{
  uint32_t cols = msg.info.width; 
  uint32_t rows = msg.info.height; 
  
  // data
  Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> map_data;
  {
    std::vector<int8_t> temp(msg.data.data(), msg.data.data()+(rows*cols));
    for(int i = 0; i < temp.size(); i++)
      temp[i] = (temp[i] == -1) ? 50 : temp[i];
    
    using ArrayXXi = Eigen::Array<int8_t, Eigen::Dynamic, Eigen::Dynamic,
      Eigen::RowMajor>;
    ArrayXXi data(rows, cols);
    data = Eigen::Map<ArrayXXi>(temp.data(), rows, cols);
    
    map_data = data.cast<double>() / 100.0;
  }

  _map = Eigen::log(map_data / (1.0 - map_data)).matrix();
  
  auto& position = msg.info.origin.position;
  _map_origin = veci<2>(-position.y/resolution - 0.5,
    -position.x/resolution - 0.5);
  _bottom_left = veci<2>(0,0);
  _top_right = veci<2>(rows-1, cols-1);
}


void
OccupancyGrid::merge(OccupancyGrid const& other,
  math::MyTransformData const& transform)
{
  vecd<2> other_map_origin(transform.translation.x, transform.translation.y);
  veci<2> other_origin = mapToGridCoordinate(other_map_origin);

  auto& other_bl = other._bottom_left;
  for(int i = other_bl[0]; i < other_bl[0]+other.height(); i++)
    for(int j = other_bl[1]; j < other_bl[1]+other.width(); j++)
    {
      veci<2> pos = other_origin + (veci<2>(i, j) - other._map_origin);
      if(update_map_size(pos))
        _map(pos(0), pos(1)) += other._map(i, j);
    }
}