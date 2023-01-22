#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include "rigid2d/diff_drive.hpp"

rigid2d::DiffDrive robot;

void update_robot_state(geometry_msgs::Twist::ConstPtr const& twist)
{
  rigid2d::Twist tw(twist->angular.z, twist->linear.x, twist->linear.y);
  robot.feedforward(tw);
}

sensor_msgs::JointState
get_joints_state_message(std::string const& left_wheel_joint,
                         std::string const& right_wheel_joint)
{
  auto wheels = robot.wheelsPosition();
  sensor_msgs::JointState msg;
  msg.header.stamp = ros::Time::now();
  
  msg.name.push_back(left_wheel_joint);
  msg.position.push_back(wheels[0]);
  msg.velocity.push_back(0);
  msg.effort.push_back(0);
  
  msg.name.push_back(right_wheel_joint);
  msg.position.push_back(wheels[1]);
  msg.velocity.push_back(0);
  msg.effort.push_back(0);

  return msg;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_turtle");
  ros::NodeHandle nh;

  double wheel_base, wheel_radius;
  std::string left_wheel_joint, right_wheel_joint;

  nh.getParam("wheel_base", wheel_base);
  nh.getParam("wheel_radius", wheel_radius);
  nh.getParam("left_wheel_joint", left_wheel_joint);
  nh.getParam("right_wheel_joint", right_wheel_joint);

  robot = rigid2d::DiffDrive(wheel_base, wheel_radius);
  auto cmd_vel_sub = nh.subscribe("cmd_vel", 100, update_robot_state);

  auto joints_pub = nh.advertise<sensor_msgs::JointState>("joint_states",
                                                          100);
  
  ros::NodeHandle _nh("~");
  double frequency;
  _nh.getParam("frequency", frequency);
  auto rate = ros::Rate(frequency);

  while(ros::ok())
  {
    auto joints_msg = get_joints_state_message(left_wheel_joint,
                                               right_wheel_joint);
    joints_pub.publish(joints_msg);
    ros::spinOnce();
    rate.sleep();
  }

}

                    
