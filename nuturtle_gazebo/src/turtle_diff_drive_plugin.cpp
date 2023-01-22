#include "nuturtle_gazebo/turtle_diff_drive_plugin.hpp"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
  void
  TurtleDrivePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL("A ROS node for Gazebo has not been initialized."
                "Unable to load plugin. Load the Gazebo system plugin"
                "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
      return;
    }

    
    this->_model = parent;

    
    _left_wheel_joint = _model->GetJoint(
        sdf->GetElement("left_wheel_joint")->Get<std::string>());
    _right_wheel_joint = _model->GetJoint(
        sdf->GetElement("right_wheel_joint")->Get<std::string>());
    if(!_left_wheel_joint || !_right_wheel_joint)
    {
      gzerr << "Unable to find right_wheel_joint or left_wheel_joint\n";
      return;
    }
    initJointsMsg();
     
    double wheel_radius, robot_base;
    if(sdf->HasElement("wheel_radius"))
      wheel_radius = sdf->GetElement("wheel_radius")->Get<double>();
    else
    {
      ROS_FATAL("wheel radius not specified in robot urdf model");
      return;
    }
    if(sdf->HasElement("robot_base"))
      robot_base = sdf->GetElement("robot_base")->Get<double>();
    else
    {
      ROS_FATAL("robot base size not specified in robot urdf model");
      return;
    }
    _robot = rigid2d::DiffDrive(robot_base, wheel_radius);

    
    if(sdf->HasElement("motor_torque_max"))
      _motor_max_torque = sdf->GetElement("motor_torque_max")->Get<double>();
    else
    {
      ROS_WARN("Motor max torque not specified in urdf model, using default:"
               " 10Nm"); 
    }

    if(sdf->HasElement("motor_rot_max"))
      _motor_max_rotation = sdf->GetElement("motor_rot_max")->Get<double>();
    else
    {
      ROS_WARN("Motor max rotation not specified in urdf model, "
               "using default: 6.35492 rad/s");
    }


    if(sdf->HasElement("encoder_pub_frequency"))
    {
      auto freq = sdf->GetElement("encoder_pub_frequency")->Get<double>();
      _encoder_pub_period = 1.0 / freq;
    }
    else
    {
      ROS_WARN("Encoder frequency not specified in urdf model, "
               "using default: 200 Hz");
      _encoder_pub_period = 1.0 / 200.0;
    }
    
    if(sdf->HasElement("encoder_topic"))
    {
      _encoder_publisher = _nh.advertise<sensor_msgs::JointState>(
          sdf->GetElement("encoder_topic")->Get<std::string>(),
          10);
    }
    else
    {
      ROS_WARN("encoder topic not specified in urdf model");
      return;
    }


    if(sdf->HasElement("cmd_vel_topic"))
    {
      _wheel_cmd_sub = _nh.subscribe(
          sdf->GetElement("cmd_vel_topic")->Get<std::string>(),
          1, // queue size
          &TurtleDrivePlugin::cmdVelCallback,
          this);
    }
    else
    {
      ROS_FATAL("cmd_vel_topic not specified in urdf model");
      return;
    }

    this->_update_connection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&TurtleDrivePlugin::onUpdate, this));
    
    _last_update_time = _model->GetWorld()->SimTime();
    ROS_INFO("TurtleDrive plugin loaded successfully");
  }

  void
  TurtleDrivePlugin::initJointsMsg()
  {
    _joints_message.name.push_back(_left_wheel_joint->GetName());
    _joints_message.position.push_back(0);
    _joints_message.velocity.push_back(0);
    _joints_message.effort.push_back(0);

    _joints_message.name.push_back(_right_wheel_joint->GetName());
    _joints_message.position.push_back(0);
    _joints_message.velocity.push_back(0);
    _joints_message.effort.push_back(0);
  }

  
  void
  TurtleDrivePlugin::onUpdate()
  {
    auto current_time = _model->GetWorld()->SimTime();
    auto dt = (current_time - _last_update_time).Double();
    
    if(dt >= _encoder_pub_period)
    {
      setJointsMsg();
      _encoder_publisher.publish(_joints_message);
      
      if(_wheel_cmd_available)
      {
        auto wheels_vel = _robot.wheelsVelocities();
        _left_wheel_joint->SetParam("fmax", 0, _motor_max_torque);
        _left_wheel_joint->SetParam("vel", 0, wheels_vel.left);
        _right_wheel_joint->SetParam("fmax", 0, _motor_max_torque);
        _right_wheel_joint->SetParam("vel", 0, wheels_vel.right);

        _wheel_cmd_available = false;

      }

      _last_update_time = current_time;
    }

  }
  
  void
  TurtleDrivePlugin::setJointsMsg()
  {
    constexpr int left_joint = 0;
    constexpr int right_joint = 1;

    _joints_message.header.stamp = ros::Time::now();
    _joints_message.position[left_joint] = _left_wheel_joint->Position();
    _joints_message.position[right_joint] = _right_wheel_joint->Position();
  }

  template <typename T>
  T clamp(T value, T min, T max)
  {
    if(value > max)
      return max;
    if(value < min)
      return min;
    return value;
  }

  void
  TurtleDrivePlugin::cmdVelCallback(geometry_msgs::Twist::ConstPtr const& twist)
  {
    rigid2d::Twist tw(twist->angular.z, twist->linear.x, twist->linear.y);
    auto wheels_vel = _robot.twistToWheels(tw);
    
    auto min = -_motor_max_rotation;
    auto max = _motor_max_rotation;
    wheels_vel.left = clamp(wheels_vel.left, min, max);
    wheels_vel.right = clamp(wheels_vel.right, min, max);

    _robot.feedforward(_robot.wheelsToTwist(wheels_vel)); 
    _wheel_cmd_available = true;
  }

}
