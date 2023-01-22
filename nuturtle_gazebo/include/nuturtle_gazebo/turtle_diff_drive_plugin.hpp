#ifndef __TURTLE_DIFF_DRIVE_PLUGIN_HPP_
#define __TURTLE_DIFF_DRIVE_PLUGIN_HPP_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <string>

#include "rigid2d/diff_drive.hpp"

namespace gazebo
{
  class TurtleDrivePlugin : public ModelPlugin
  {
  private:
    double _motor_max_torque, _motor_max_rotation, _encoder_pub_period;
    bool _wheel_cmd_available;
    common::Time _last_update_time;
    physics::ModelPtr _model;
    physics::JointPtr _left_wheel_joint;
    physics::JointPtr _right_wheel_joint;
    event::ConnectionPtr _update_connection;
   
    ros::NodeHandle _nh;
    ros::Subscriber _wheel_cmd_sub;
    ros::Publisher _encoder_publisher;
    sensor_msgs::JointState _joints_message;
    
    rigid2d::DiffDrive _robot;

    void onUpdate();

    void cmdVelCallback(geometry_msgs::Twist::ConstPtr const& twist);

    void setJointsMsg();
    void initJointsMsg();
  public:
    TurtleDrivePlugin() : _wheel_cmd_available{false} {};
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
  };

  GZ_REGISTER_MODEL_PLUGIN(TurtleDrivePlugin)
}
#endif
