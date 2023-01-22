#include "nuslam/analysis.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


void publish_ground_truth_cylinders_markers(ros::NodeHandle& nh, 
                                            ros::Publisher& true_landmarks_pub)
{
  constexpr auto gazebo_world_props_service = "/gazebo/get_world_properties";
  ros::service::waitForService(gazebo_world_props_service);
  
  ros::ServiceClient gazebo_world_props_client = nh.serviceClient<
    gazebo_msgs::GetWorldProperties>(gazebo_world_props_service);

  gazebo_msgs::GetWorldProperties world_props_srv;
  
  if(!gazebo_world_props_client.call(world_props_srv))
  {
    ROS_ERROR("Failed to call service %s", gazebo_world_props_service);
    return;
  }

  auto& res = world_props_srv.response;
  if(res.success == true)
  {
    visualization_msgs::MarkerArray cylinders;
    auto get_model = nh.serviceClient<gazebo_msgs::GetModelState>(
        "/gazebo/get_model_state");
    for(std::string& model : res.model_names)
    {
      if(model.substr(0, 8) == "cylinder")
        add_cylinder(model, cylinders.markers, get_model);
    }
    true_landmarks_pub.publish(cylinders);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "analysis");
  ros::NodeHandle nh;
  ros::Publisher true_landmarks_pub = 
    nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array", 5);
  
  auto r = ros::Rate(1); 
  while(ros::ok())
  {
    publish_ground_truth_cylinders_markers(nh, true_landmarks_pub);
    ros::spinOnce();
    r.sleep();
  }

  //ros::spin();
}
