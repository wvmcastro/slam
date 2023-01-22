#ifndef __OCCUPANCY_GRID_HPP
#define __OCCUPANCY_GRID_HPP

#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <string>

#include <math_utils/math.hpp>
using math::veci;
using math::vecd;

using matdrj = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

class OccupancyGrid
{
public:
  constexpr static double p_occupied = 0.97;
  constexpr static double p_free = 0.12;

  constexpr static double log_occupied = math::probability_to_log_odds(p_occupied);
  constexpr static double log_free = math::probability_to_log_odds(p_free);

  OccupancyGrid(double map_size, 
               double res)
    : resolution{res}
    , _bottom_left{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()}
    , _top_right{-1, -1}
  {

    uint size = std::ceil(map_size / res);
    if(size % 2 == 0) size += 1;
    _map = matdrj(size, size);
    _map << Eigen::MatrixXd::Constant(size, size, 0);

    _map_origin = veci<2>(size/2, size/2);
  }
  
  OccupancyGrid(nav_msgs::OccupancyGrid const& msg);

  double resolution;

  int size() const
  {
    return _map.rows();
  }

  void updateMap(Eigen::Vector2d const&, Eigen::Vector2d const&, bool);
  veci<2> mapToGridCoordinate(vecd<2> const&) const;
  nav_msgs::OccupancyGrid gridMapMessage(std::string const&, ros::Time const&) const;
  void merge(OccupancyGrid const& other, math::MyTransformData const& transform);

private:
  veci<2> _bottom_left, _top_right;
  matdrj _map;
  veci<2> _map_origin;

  bool update_map_size(veci<2> const&);
  bool update_extremes(int& lower_bound, int& upper_bound, int value)
  {
    if(value < 0)
      return false;
    
    if(value >= _map.rows())
      return false;

    if(value < lower_bound)
      lower_bound = value;
    else if(value > upper_bound)
      upper_bound = value;
    
    return true;
  }

  int width() const
  {
    return _top_right[1] - _bottom_left[1] + 1;
  }
  
  int height() const
  {
    return _top_right[0] - _bottom_left[0] + 1;
  }
};

#endif