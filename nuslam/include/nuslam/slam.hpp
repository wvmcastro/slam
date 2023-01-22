#ifndef __SLAM_HPP
#define __SLAM_HPP

#include "filter.hpp"
#include <nuslam/TurtleMap.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <string>
#include <vector>
#include <tuple>
#include <nuslam/SlamMap.h>
#include <nuslam/MapExchangeSrv.h>
#include <nuslam/MapExchangeCommand.h>
#include <occupancy_grid_map/MapBuilderAndPublisher.hpp>
#include <pointcloud_utils/registration_pipeline.hpp>

#include <string>

using pointcloud::registration::RegistrationPipeline;

struct Params
{
  float transform_update_delay;
  double encoder_frequency;
  std::string robot_ns;
  bool debug;
};

#define T_ float
class Slam
{
public:
  // landmark dim
  constexpr static int P = 2;
  
  // control dim
  constexpr static int M = 2;
  
  
  Slam(nuslam::Filter<M>& filter, std::string const& robot_name_, 
    bool known_initial_pose=false):
    robot_name{robot_name_},
    _is_robot_initialized{false},
    _filter{filter}, 
    _encoder_flag{false}, 
    _measurement_flag{false},
    _registration{},
    _known_initial_pose{known_initial_pose}
  {
    init();

    auto x0 = _filter.state();
  }

  const std::string robot_name;

  void run(void);
private:
  bool _is_robot_initialized;
  nuslam::Filter<M>& _filter;
  std::mutex _filter_mutex;
  ros::NodeHandle _nh;
  std::map<std::string, ros::Subscriber> _subscribers;
  std::map<std::string, ros::Publisher> _publishers;
  tf2_ros::TransformBroadcaster _odom_map_tf_broadcaster;
  tf2_ros::Buffer _tf2_buffer;
  tf2_ros::TransformListener _tf2_listener{_tf2_buffer};
  std::vector<ros::ServiceServer> _services;

  std::map<std::string, ros::Time> _communication_register;

  Params _params;
  std::string _map_frame;

  vecd<M> _encoder_reading;
  bool _encoder_flag;

  bool _measurement_flag;
  std::vector<std::tuple<uint16_t, vecd<2>>> _landmark_measurements; 

  RegistrationPipeline _registration;

  bool _known_initial_pose;

  void init();
  void initSubscribers();
  void initPublishers();
  void initServices();

  void landmarksCallback(nuslam::TurtleMap::ConstPtr const&);
  void encoderCallback(sensor_msgs::JointState::ConstPtr const&);
  void publishMapOdomTransform();
  void publishPose();
  void publishFeatureMap();
  bool seifMapExchangeResponse(nuslam::MapExchangeSrv::Request &req,
                               nuslam::MapExchangeSrv::Response &res);
  
  void makeMapExchangeRequest(nuslam::MapExchangeCommand::ConstPtr const&);
  void doMapIntegration(nuslam::MapExchangeSrvResponse const&);
  void transformMap(math::MyTransformData const& t, math::vecd<>& map,
    int landmark_dim);
  
  void sendExchangeCommandToGridMap(std::string const&,
    math::MyTransformData const&);

  bool getTransform(Eigen::Ref<const vecd<>> source,
    Eigen::Ref<const vecd<>> target, math::MyTransformData& transform) const;
  
  bool getTransform(std::string const& other_name,
    math::MyTransformData& transform) const;
  
  bool getTransform(Eigen::Ref<const vecd<>> other_data,
    std::string const& other_robot_name,
    math::MyTransformData& transform);
};

namespace slam
{

pointcloud::PointCloud mapToPointCloud(Eigen::Ref<math::vecd<> const> & map);

}

#endif
