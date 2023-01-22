#include <occupancy_grid_map/OccupancyGrid.hpp>
#include <occupancy_grid_map/MapBuilderAndPublisher.hpp>

constexpr char exchange_map_command_topic[] = "/grid_map_exchange_command";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_grid_map");
  ros::NodeHandle nh{};

  std::string lidar_topic; 
  nh.getParam("laser_topic", lidar_topic);
  std::string map_frame{argv[1]};

  float cell_size;
  ros::param::get("~cell_size", cell_size);
  OccupancyGrid grid{20, cell_size};

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tf2_listener{buffer};
  MapBuilderAndPublisher map_publisher{map_frame, grid, buffer, nh};

  ros::Subscriber laser_sub;
  laser_sub = nh.subscribe(lidar_topic, 
                           1, 
                           &MapBuilderAndPublisher::publishMap, 
                           &map_publisher);
  
  ros::Subscriber exchange_map_sub;
  exchange_map_sub = nh.subscribe(exchange_map_command_topic,
                                  1,
                                  &MapBuilderAndPublisher::exchangeMap,
                                  &map_publisher);
  
  ros::spin();
}
