#include "nunavigation/guide.hpp"
#include <misc_utils/misc.hpp>

void 
Guide::init(const std::string& map_topic, const std::string& pose_topic)
{
  _map_sub = _nh.subscribe(map_topic,
                           1,
                           &Guide::mapCallback,
                           this);
  _pose_sub = _nh.subscribe(pose_topic,
                            1,
                            &Guide::poseCallback,
                            this);
}

void
Guide::mapCallback(nav_msgs::OccupancyGrid::ConstPtr const& msg)
{
  if(_map_frame == "")
    _map_frame = msg->header.frame_id;

  uint32_t w = msg->info.width;
  uint32_t h = msg->info.height;

  _map = Eigen::Map<const map>(msg->data.data(), h, w);

  _map_metadata = msg->info;
}

void
Guide::poseCallback(geometry_msgs::PoseWithCovarianceStamped::ConstPtr const& msg)
{
  _position[0] = msg->pose.pose.position.x;
  _position[1] = msg->pose.pose.position.y;
}

std::pair<math::vecf<2>, std::string>
Guide::nextGoal()
{
  uint32_t neighborhood_padding = 
    get_neighborhood_padding(_map_metadata.resolution);

  int inf = std::numeric_limits<int>::infinity(); 
  math::veci<2> goal{inf, inf};
  auto robot_pos = mapToGrid(_position);

  uint32_t min_distance{std::numeric_limits<uint32_t>::max()};

  int padding = padding_size / _map_metadata.resolution;  
  for(int i = padding; i < _map.rows()-padding; i++)
    for(int j = padding; j < _map.cols()-padding; j++)
    {
      if(_map(i, j) == -1) // i.e. unknown
      {
        uint32_t d = math::distance_l1(robot_pos, math::veci<2>{i, j});
        if(d < min_distance)
        {
          if(check_free_neighborhood(i, j, neighborhood_padding) == true)
          {
            min_distance = d;
            goal = math::veci<2>{i, j};
          }
        }
      }
    }

  return {gridToMap(goal), _map_frame};
}

math::veci<2> 
Guide::mapToGrid(math::vecf<2> const& p) const
{
  math::vecf<2> map_origin{_map_metadata.origin.position.x,
                           _map_metadata.origin.position.y};

  float r = _map_metadata.resolution; 
  auto p_map = (p - map_origin).array()/r + 0.5;

  return math::veci<2>{p_map[1], p_map[0]};
}

math::vecf<2>
Guide::gridToMap(math::veci<2> const& p) const
{
  math::vecf<2> p_map = p.cast<float>() * _map_metadata.resolution;
  swap(p_map);
  auto& mo = _map_metadata.origin.position;
  return p_map + math::vecf<2>{mo.x, mo.y};
}

uint32_t 
Guide::get_neighborhood_padding(double grid_resolution) const
{
  constexpr double cylinder_radius = 0.08;
  constexpr double inflation_radius = 0.5;
  uint32_t padding = 
    (uint32_t) ((inflation_radius + cylinder_radius) / grid_resolution);
  return padding;
}

bool 
Guide::check_free_neighborhood(uint32_t row, 
                               uint32_t col, 
                               uint32_t neighborhood_padding)
{
  uint32_t p = neighborhood_padding;
  
  if(row-p < 0 || row+p >= _map.rows()) // out of map range 
    return false;
  
  if(col-p < 0 || col+p >= _map.cols()) // out of map range 
    return false;
  
  for(int i = row-p; i < row+p; i++)
    for(int j = col-p; j < col+p; j++)
    {
      if(_map(i,j) >= occupied_threshold)
        return false;
    }
  
  return true;
}
