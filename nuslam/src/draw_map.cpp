#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nuslam/TurtleMap.h>
#include <math_utils/math.hpp>

class MapDrawer
{
public:
  MapDrawer(std::string const& landmarks_topic)
  {
    _map_pub = _nh.advertise<visualization_msgs::Marker>(
        "visualization_marker", 20);
    _landmarks_sub = _nh.subscribe(landmarks_topic, 10, 
        &MapDrawer::landmarks_callback, this);
  }
private:
  ros::NodeHandle _nh;
  ros::Subscriber _landmarks_sub;
  ros::Publisher _map_pub;

  void landmarks_callback(nuslam::TurtleMap::ConstPtr const& landmarks)
  {
    int n = landmarks->range.size();
    auto const& l = landmarks;
    for(int i = 0; i < n; i++)
    {
      double point[2] = {l->range[i], l->bearing[i]};
      math::polarToCartesian(point);
      auto marker_msg = get_marker_message(i, point, 
        l->radius[i], l->header.frame_id);
      _map_pub.publish(marker_msg);
    }
  }

  visualization_msgs::Marker get_marker_message(int id,
                                                double* point,
                                                float radius,
                                                std::string const& frame_id)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.ns = "visualization/nuslam/detected_landmarks";
    marker.lifetime = ros::Duration(0.2);
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = point[0];
    marker.pose.position.y = point[1];
    marker.pose.orientation.w = 1;
    marker.scale.x = marker.scale.y = 2*radius;
    marker.scale.z = 0.45;
    
    marker.color.r = 1.0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;

    return marker;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_map");
  ros::NodeHandle nh;

  std::string landmarks_topic;
  
  nh.getParam("landmarks_topic", landmarks_topic);
  MapDrawer cylinders_drawer(landmarks_topic);
  ros::spin();
}
