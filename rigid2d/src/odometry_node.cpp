#include "rigid2d/diff_drive.hpp"
#include "rigid2d/set_pose.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h> // sensor_msgs::JointState
#include <geometry_msgs/Pose2D.h> // reset pose server message
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // tf2::toMsg
#include <string>


class OdometryNode
{
private: 
  bool _publish_tf;
  std::string _odom_frame_id, _body_frame_id;
  static constexpr char _left_wheel_joint[] = "wheel_left_joint"; 
  static constexpr char _right_wheel_joint[] = "wheel_right_joint";
  int _left_joint_index, _right_joint_index;
  double _wheel_base, _wheel_radius;
  rigid2d::DiffDrive _robot;
  
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  ros::Subscriber _joint_sub;
  ros::Publisher _odom_publisher;
  tf2_ros::TransformBroadcaster _odom_tf_broadcaster;
  
  // The doc says that the service server will be shutdown once all instaces
  // of the ServiceServer object are destroyed. Although of the destructor
  // do not call the shutdown method (?)
  ros::ServiceServer _reset_pose_server;


public:
  OdometryNode()
  {
    _private_nh = ros::NodeHandle("~");
    _reset_pose_server = _nh.advertiseService("set_pose",
                                              &OdometryNode::setPose,
                                              this);
    _odom_publisher = _nh.advertise<nav_msgs::Odometry>("odom", 10);

    _left_joint_index = _right_joint_index = -1;
  }
  
  void init()
  {
    _private_nh.getParam("odom_frame_id", _odom_frame_id);
    _private_nh.getParam("body_frame_id", _body_frame_id);
    _private_nh.getParam("publish_tf", _publish_tf);
    
    _nh.getParam("wheel_base", _wheel_base);
    _nh.getParam("wheel_radius", _wheel_radius);

    _robot = rigid2d::DiffDrive(_wheel_base, _wheel_radius);
    
    std::string joints_states_topic;
    _private_nh.getParam("joints_topic", joints_states_topic);
    _joint_sub = _nh.subscribe("joint_states", 
                               100, 
                               &OdometryNode::jointsCallback,
                               this);
  }

  void
  jointsCallback(sensor_msgs::JointState::ConstPtr const& js)
  {
    // Header header

    // string[] name
    // float64[] position
    // float64[] velocity
    // float64[] effort

    // all the arrays in the joint state message are std::vector

    setJointsIndexes(js);

    auto left_wheel_encoder = js->position[_left_joint_index];
    auto right_wheel_encoder = js->position[_right_joint_index];

    _robot.updateOdometry(left_wheel_encoder, right_wheel_encoder);
    
    auto currentTime = ros::Time::now();

    if(_publish_tf)
      broadcastTF(currentTime);
    publishOdometry(currentTime);
  }
  
  void setJointsIndexes(sensor_msgs::JointState::ConstPtr const& js)
  {
    if(_left_joint_index == -1)
    {
      int i = 0;
      int k = 0;
      for(auto& joint : js->name)
      {
        if(joint == _left_wheel_joint)
        {
          _left_joint_index = k;
          i++;
        }
        else if(joint == _right_wheel_joint)
        {
          _right_joint_index = k;
          i++;
        }

        if(i == 2)
          break;
        
        k++;
      }
    }
  }
  
  void publishOdometry(ros::Time const& current_time)
  {
    auto pose = _robot.pose();
    
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = _odom_frame_id;
    
    // position in the odom frame
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose.theta);
    odom_quat.normalize();
    odom.pose.pose.orientation = tf2::toMsg(odom_quat);
    
    // twist in the body frame
    odom.child_frame_id = _body_frame_id;
    auto tw = _robot.wheelsToTwist(_robot.wheelsVelocities());
    odom.twist.twist.linear.x = tw.v.x;
    odom.twist.twist.linear.y = tw.v.y;
    odom.twist.twist.angular.z = tw.w;
    
    _odom_publisher.publish(odom);
  }
  
  void broadcastTF(ros::Time const& current_time)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = _odom_frame_id;
    transform.child_frame_id = _body_frame_id;
    
    auto pose = _robot.pose();
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, pose.theta);
    odom_quat.normalize();
    transform.transform.rotation.x = odom_quat.x();
    transform.transform.rotation.y = odom_quat.y();
    transform.transform.rotation.z = odom_quat.z();
    transform.transform.rotation.w = odom_quat.w();
      
    _odom_tf_broadcaster.sendTransform(transform);
  }

  bool setPose(rigid2d::set_pose::Request& req,
               rigid2d::set_pose::Response& res) 
  {
    _robot.reset(req.pose.x, req.pose.y, req.pose.theta);
    res.success = true;
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");

  auto node = OdometryNode();
  node.init();

  ros::spin();
}
