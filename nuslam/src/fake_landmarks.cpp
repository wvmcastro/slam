#include <ros/ros.h>
#include <string>
#include <vector>
#include <random>
#include <cmath>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <visualization_msgs/MarkerArray.h>
#include <nuslam/TurtleMap.h>
#include <rigid2d/rigid2d.hpp>

#include <math_utils/math.hpp>

constexpr float cylinder_radius = 0.08;
constexpr float base_footprint_to_base_scan_x_offset = -0.032;

std::mt19937& get_random()
{
  static std::random_device rd{};
  static std::mt19937 mt{rd()}; 

  return mt;
}

std::vector<std::string> 
getLandmarkNames(ros::NodeHandle& nh)
{
  constexpr auto service_name = "/gazebo/get_world_properties";
  
  ros::service::waitForService(service_name);
  auto gazebo_world_props_client = nh.serviceClient<
    gazebo_msgs::GetWorldProperties>(service_name);

  gazebo_msgs::GetWorldProperties world_props_srv;
  if(!gazebo_world_props_client.call(world_props_srv))
  {
    ROS_ERROR("Failed to call service %s", service_name);
    return std::vector<std::string>();
  }

  auto& res = world_props_srv.response;
  if(res.success == true)
  {
    std::vector<std::string> landmark_names;
    landmark_names.reserve(res.model_names.size());
    for(auto& model : res.model_names)
      if(model.substr(0, 8) == "cylinder")
        landmark_names.push_back(model);
    return landmark_names;
  }
  else
    return std::vector<std::string>();
}

std::pair<std::vector<int>, std::vector<std::array<double, 2>>>
getLandmarksPositions(std::vector<std::string> const& landmark_names,
                      ros::ServiceClient get_model,
                      const std::string& robot_name)
                        
{
  gazebo_msgs::GetModelState model_state_srv;
  model_state_srv.request.relative_entity_name = robot_name + "::base_footprint";

  
  std::vector<std::array<double, 2>> landmark_positions;
  landmark_positions.reserve(landmark_names.size()); 
  std::vector<int> landmarks_ids{};
  landmarks_ids.reserve(landmark_names.size());
  int k = 0;
  for(auto& landmark : landmark_names)
  {
    model_state_srv.request.model_name = landmark;
    model_state_srv.response.success = false;
    get_model.call(model_state_srv);
    if(model_state_srv.response.success == true)
    {
      model_state_srv.response.status_message;
      auto& position = model_state_srv.response.pose.position;    
      landmark_positions.emplace_back(std::array{position.x, position.y});
      landmarks_ids.push_back(k);
    }
    else
    {
      ROS_ERROR_STREAM(model_state_srv.response.status_message);
    }
    k++;
  }
  return std::make_pair(landmarks_ids, landmark_positions);
}

void
changeLandmarksCoordFrame(std::vector<std::array<double, 2>>& landmarks_in_robot_frame)
{
  // changes from robot frame to
  // lidar sensor frame
  for(auto& landmark : landmarks_in_robot_frame)
    landmark[0] -= base_footprint_to_base_scan_x_offset;
}


nuslam::TurtleMap
getLandmarksMessage(const std::vector<int>& landmarks_ids, 
                    std::vector<std::array<double, 2>>& landmarks_positions, 
                    float range_max,
                    float sensor_noise,
                    std::string robot_name)
{
  nuslam::TurtleMap landmarks_msg;
  landmarks_msg.header.frame_id = robot_name + "/base_scan";
  std::normal_distribution<float> noise(0, sensor_noise);

  math::cartesianToPolar(landmarks_positions);
  int k = 0;
  for(auto& landmark : landmarks_positions)
  {
    constexpr int r = 0;
    constexpr int theta = 1;
    if(landmark[r] < range_max)
    {
      landmarks_msg.id.push_back(landmarks_ids[k]);
      landmarks_msg.range.push_back(landmark[r] + noise(get_random()));
      landmarks_msg.bearing.push_back(
        landmark[theta] + noise(get_random())
        );
      landmarks_msg.radius.push_back(cylinder_radius);
    }
    k++;
  }

  landmarks_msg.header.stamp = ros::Time::now();
  return landmarks_msg;
}

// get the landmarks relative to base_scanner
// params: freq, radius, noise
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_landmarks");
  ros::NodeHandle nh;
  std::string landmarks_topic;
  nh.getParam("landmarks_topic", landmarks_topic);
  
  auto landmarks_pub = nh.advertise<nuslam::TurtleMap>(landmarks_topic, 1);

  float max_range;
  nh.getParam("laser_max_range", max_range); 
  float range_noise;
  nh.getParam("laser_stddev", range_noise); 
  float publish_frequency; 
  nh.getParam("laser_update_rate", publish_frequency);
  ros::Rate r{publish_frequency};
  
  auto landmark_names = getLandmarkNames(nh);

  constexpr auto get_model_service = "/gazebo/get_model_state";
  ros::service::waitForService(get_model_service);
  ros::ServiceClient get_model = nh.serviceClient<gazebo_msgs::GetModelState>(
      get_model_service);
  
  std::string robot_name;
  nh.getParam("robot_name", robot_name);
  while(ros::ok())
  {
    auto landmarks = getLandmarksPositions(landmark_names, get_model, robot_name);
    
    changeLandmarksCoordFrame(landmarks.second);

    auto landmarks_msg = getLandmarksMessage(landmarks.first, 
                                             landmarks.second,
                                             max_range,
                                             range_noise,
                                             robot_name);
    landmarks_pub.publish(landmarks_msg);

    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
}
