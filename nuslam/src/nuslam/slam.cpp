#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nuslam/slam.hpp>
#include <math_utils/math.hpp>
#include <nuslam/nuslam_system.hpp>
#include <nuslam/seif.hpp>
#include <misc_utils/misc.hpp>

using math::vec;
using math::mat;

#include <cmath>

constexpr char map_exchange_command_topic[] = "/map_exchange_command";

void
Slam::init(void)
{
  _nh.getParam("debug", _params.debug);
  
  float laser_update_rate;
  _nh.getParam("laser_update_rate", laser_update_rate);
  _params.transform_update_delay = 1.0 / laser_update_rate;

  _nh.getParam("encoder_frequency", _params.encoder_frequency);

  _nh.getParam("robot_name", _params.robot_ns);
  _map_frame = _params.robot_ns + "/map";
  
  initSubscribers();
  initPublishers();
  initServices();
}

void 
Slam::initSubscribers(void)
{
  std::string str_buffer;
  _nh.getParam("landmarks_topic", str_buffer);
  _subscribers["landmarks"] = _nh.subscribe(str_buffer, 1, 
      &Slam::landmarksCallback, this);
  
  _subscribers["encoder"] = _nh.subscribe("joint_states", 10, 
    &Slam::encoderCallback, this);
  
  _subscribers["mapExchange"] = _nh.subscribe(map_exchange_command_topic, 1,
    &Slam::makeMapExchangeRequest, this);
}

void
Slam::initPublishers()
{
  _publishers["pose"] = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    "slam/pose", 1);

  _publishers["featureMap"] = _nh.advertise<nuslam::SlamMap>("slam/map", 1);

  _publishers["gridMapExchangeCommand"] = _nh.advertise<
    geometry_msgs::TransformStamped>("/grid_map_exchange_command", 1);
}

void 
Slam::initServices()
{
  if(dynamic_cast<nuslam::SEIFSlamDiffDrive*>(&_filter) != nullptr)
  {
    ROS_INFO_STREAM("Using SEIF Filter");
    _services.emplace_back(_nh.advertiseService("map_exchange", 
                                                &Slam::seifMapExchangeResponse, 
                                                this));
  }
}

bool
Slam::seifMapExchangeResponse(nuslam::MapExchangeSrv::Request &req,
                              nuslam::MapExchangeSrv::Response &res)
{
  nuslam::SEIFSlamDiffDrive* seif;
  seif = dynamic_cast<nuslam::SEIFSlamDiffDrive*>(&_filter);
  res = seif->getExchangeMapData();
  res.data.robot_id = robot_name;
  return true;
}

void
Slam::landmarksCallback(nuslam::TurtleMap::ConstPtr const& msg)
{
  int n = msg->range.size();
  _landmark_measurements.resize(n);
  for(int i = 0; i < n; i++)
  {
    _landmark_measurements[i] = std::make_pair(
        msg->id[i],
        vec<double, 2>(msg->range[i], msg->bearing[i])
        );
  }
  _measurement_flag = true;
}

void 
Slam::encoderCallback(sensor_msgs::JointState::ConstPtr const& encoder_msg)
{
  static vecd<2> previous_input{0, 0};
  constexpr int left_wheel = 0;
  constexpr int right_wheel = 1;
  _encoder_reading(0) = encoder_msg->position[left_wheel];
  _encoder_reading(1) = encoder_msg->position[right_wheel];

  vecd<2> diff_abs = (previous_input - _encoder_reading).cwiseAbs();
  double input_magnitude = diff_abs.maxCoeff();

  previous_input = _encoder_reading;
  _encoder_flag = true;
}

void
Slam::makeMapExchangeRequest(
  nuslam::MapExchangeCommand::ConstPtr const& command)
{
  std::string other{""};
  if(command->peer1 == robot_name) 
    other = command->peer2;
  else if(command->peer2 == robot_name)
    other = command->peer1;

  if(other == "")
    return;

  auto it = _communication_register.find(other);
  if(it != _communication_register.end())
  {
    ros::Duration dt = ros::Time::now() - it->second;
    ROS_WARN_STREAM("dt: " << dt.toSec());
    if(dt.toSec() < 60)
      return;
  }

  std::thread t([=](){
    std::string service = "/" + other + "/map_exchange";
    ros::ServiceClient client = this->_nh.serviceClient<nuslam::MapExchangeSrv>(
      service
    );

    nuslam::MapExchangeSrv srv;
    srv.request.robot_id = this->robot_name;
    if(client.call(srv) == false){
      ROS_ERROR_STREAM("Map request to " << service << " error.");
      return;
    }
    ROS_INFO_STREAM("Map request to " << service << " success.");
    doMapIntegration(srv.response);
  });
  t.detach();
}

void 
Slam::doMapIntegration(nuslam::MapExchangeSrvResponse const& response)
{
  nuslam::seif::Data otherData =
    nuslam::SEIFSlamDiffDrive::getMapDataFromExchangeMessage(response);


  constexpr int N = 3;
  math::MyTransformData transform; 
  if(getTransform(otherData.state,
                  response.data.robot_id,
                  transform) == false)
  {
    return;
  }
  
  _communication_register[response.data.robot_id] = ros::Time::now();

  sendExchangeCommandToGridMap(response.data.robot_id, transform);

  transformMap(transform, otherData.state, 2);
  ROS_WARN_STREAM("[" << robot_name << "] Transform: (" 
    << transform.translation.x << ", " << transform.translation.y 
    << ") theta: " << transform.rotation);

  nuslam::SEIFSlamDiffDrive::transformInfoData(otherData, transform);
  {
    auto* seif = dynamic_cast<nuslam::SEIFSlamDiffDrive*>(&_filter);
    std::lock_guard<std::mutex> lk{_filter_mutex};
    seif->integrateMap(otherData);
  }
}

bool
Slam::getTransform(Eigen::Ref<const vecd<>> other_state,
  std::string const& other_robot_name,
  math::MyTransformData& transform)
{
  if(_known_initial_pose)
    return getTransform(other_robot_name, transform);

  vecd<> state;
  {
    std::lock_guard<std::mutex> lk{_filter_mutex};
    state = _filter.state();
  }
  
  constexpr int N = 3;
  return getTransform(other_state,
                      state.segment(N, state.size() - N),
                      transform);
}

bool
Slam::getTransform(std::string const& other_name,
  math::MyTransformData& transform) const
{
  transform = misc::getTransform(robot_name + "/map", 
    other_name + "/map", _tf2_buffer, ros::Time::now(),
    ros::Duration(_params.transform_update_delay));

  return true;
}

bool
Slam::getTransform(Eigen::Ref<const vecd<>> source,
  Eigen::Ref<const vecd<>> target, math::MyTransformData& transform) const
{
  pointcloud::PointCloud source_pointcloud = slam::mapToPointCloud(source);

  pointcloud::PointCloud target_pointcloud = slam::mapToPointCloud(target);

  auto transform_result = _registration.evaluate(
    source_pointcloud, target_pointcloud);

  if(std::get<0>(transform_result) == false)
  {
    ROS_ERROR_STREAM("A correct transform could not be obtained.");
    return false;
  }
  
  ROS_WARN_STREAM("A correct transform was found.");

  math::mat3d transform_matrix = std::get<1>(transform_result);
  transform.translation.x = transform_matrix(0,2);
  transform.translation.y = transform_matrix(1,2);
  transform.rotation = std::atan2(transform_matrix(1,0), transform_matrix(0,0));

  return true;
}

void
Slam::sendExchangeCommandToGridMap(std::string const& other_robot_name,
  math::MyTransformData const& transform)
{
  ROS_WARN_STREAM("Grid map exchange command from " << _params.robot_ns << 
    " to " << other_robot_name);

  geometry_msgs::TransformStamped transform_msg;
  transform_msg.header.frame_id = other_robot_name + "/map";
  transform_msg.child_frame_id = _params.robot_ns + "/map";

  transform_msg.transform.translation.x = transform.translation.x;
  transform_msg.transform.translation.y = transform.translation.y;
  auto rotation = math::headingToQuaternion(transform.rotation);
  transform_msg.transform.rotation.x = rotation.x();
  transform_msg.transform.rotation.y = rotation.y();
  transform_msg.transform.rotation.z = rotation.z();
  transform_msg.transform.rotation.w = rotation.w();

  _publishers["gridMapExchangeCommand"].publish(transform_msg);
}

void
Slam::transformMap(math::MyTransformData const& t, vecd<>& map, int landmark_dim)
{

  vecd<2> d{t.translation.x, t.translation.y};
  matd<2> rot = math::rotationMatrix2D(t.rotation);

  auto* seif = dynamic_cast<nuslam::SEIFSlamDiffDrive*>(&_filter);
  for(int i = 0; i < map.size(); i+=landmark_dim)
    map.segment(i, landmark_dim) = rot*map.segment(i, landmark_dim) + d;
}

void
Slam::publishMapOdomTransform()
{
  constexpr float delay = 1.0 / 30;
  static std::string odom_frame = _params.robot_ns + "/odom";
  static std::string base_footprint_frame = _params.robot_ns + "/base_footprint";

  geometry_msgs::TransformStamped odom_base_tf_msg = _tf2_buffer.lookupTransform(
    odom_frame, base_footprint_frame, ros::Time::now(), ros::Duration(delay));
  tf2::Stamped<tf2::Transform> odom_base_tf;
  tf2::fromMsg(odom_base_tf_msg, odom_base_tf);

  auto filter_pose = _filter.state().head<3>();
  tf2::Vector3 position{filter_pose[1], filter_pose[2], 0};
  tf2::Quaternion orientation = math::headingToQuaternion(filter_pose[0]);
  tf2::Transform map_base_tf{orientation, position};

  tf2::Stamped<tf2::Transform> map_odom_tf(
    map_base_tf * odom_base_tf.inverse(),
    ros::Time::now() + ros::Duration(_params.transform_update_delay), // Hack stolen form GMapping Package
    _map_frame);

  geometry_msgs::TransformStamped map_odom_tf_msg = tf2::toMsg(map_odom_tf);
  map_odom_tf_msg.child_frame_id = odom_frame;
  _odom_map_tf_broadcaster.sendTransform(map_odom_tf_msg);
}

void
Slam::run(void)
{
  ros::Rate loop_rate{ _params.encoder_frequency };
  while(ros::ok()) {
    ros::spinOnce(); // run all callbacks that have data waiting in the queue
    
    _is_robot_initialized |= (_encoder_flag && _measurement_flag);
    if(_is_robot_initialized == false)
      continue;
    {
      std::lock_guard<std::mutex> lk{_filter_mutex};
        if(_encoder_flag == true) // prediction step
        {
            _filter.predict(_encoder_reading);
        }

        if(_measurement_flag)
        {
          _filter.update(_landmark_measurements);
          
          publishMapOdomTransform();
          publishPose();
          publishFeatureMap();
        }
      }
    _encoder_flag = _measurement_flag = false;
    loop_rate.sleep();
  }
}

void
Slam::publishPose()
{
  static geometry_msgs::PoseWithCovarianceStamped pose_msg{};
  pose_msg.header.frame_id = _map_frame;

  auto state = _filter.state();
  pose_msg.pose.pose.position.x = state[1];
  pose_msg.pose.pose.position.y = state[2];

  auto quat = math::headingToQuaternion(state[0]);
  pose_msg.pose.pose.orientation.x = quat.x();
  pose_msg.pose.pose.orientation.y = quat.y();
  pose_msg.pose.pose.orientation.z = quat.z();
  pose_msg.pose.pose.orientation.w = quat.w();
  
  constexpr int x = 0;
  constexpr int y = 1;
  auto cov_xy = _filter.poseCovariance().block(1,1,2,2);
  auto& cov_msg = pose_msg.pose.covariance;
  cov_msg[math::rowColIndexTo1D(x,x,6)] = cov_xy(x,x);
  cov_msg[math::rowColIndexTo1D(x,y,6)] = cov_xy(x,y);
  cov_msg[math::rowColIndexTo1D(y,y,6)] = cov_xy(y,y);
  cov_msg[math::rowColIndexTo1D(y,x,6)] = cov_xy(y,x);
  constexpr int heading = 5;
  auto cov = _filter.poseCovariance();
  cov_msg[math::rowColIndexTo1D(x, heading, 6)] = cov(x, 0);
  cov_msg[math::rowColIndexTo1D(heading, x, 6)] = cov(0, x);
  cov_msg[math::rowColIndexTo1D(y, heading, 6)] = cov(y, 0);
  cov_msg[math::rowColIndexTo1D(heading, y, 6)] = cov(0, y);
  cov_msg[math::rowColIndexTo1D(heading, heading, 6)] = cov(0, 0);

  _publishers["pose"].publish(pose_msg);
}

void
Slam::publishFeatureMap()
{
  static nuslam::SlamMap msg;
  constexpr int N = nuslam::NuslamSystem::N;
  
  auto state = _filter.state();
  int n = (state.size() - N) / 2;
  msg.x.resize(n);
  msg.y.resize(n);
  msg.covariance.resize(n);
  for(int m = 0; m < n; m++)
  {
    msg.x[m] = state(N + 2*m);
    msg.y[m] = state(N + 2*m + 1);

    auto covariance = _filter.landmarkCovariance(m);
    msg.covariance[m].layout = covariance.IsRowMajor;
    msg.covariance[m].data = std::vector<double>(covariance.data(), 
      covariance.data()+covariance.size());
  }

  _publishers["featureMap"].publish(msg);
}

namespace slam
{

pointcloud::PointCloud
mapToPointCloud(Eigen::Ref<math::vecd<> const> & map)
{
  using pointcloud::PointCloud;
  using pointcloud::Point;

  constexpr int landmark_dim = 2;

  PointCloud cloud(map.size() / landmark_dim);

  int i = 0;
  for(auto& point : cloud)
  {
    point = map.segment<landmark_dim>(i);
    i += landmark_dim;
  }

  return cloud;
}
  
}