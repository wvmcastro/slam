#include <string>
#include <random>
#include <functional> // std::bind std::ref

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include "rigid2d/diff_drive.hpp"

class TubeWorldSimulator : public rigid2d::DiffDriveSimulator
{
public:
  TubeWorldSimulator()
  {
    _private_nh = ros::NodeHandle("~");
    double wheel_base, wheel_radius;
    _nh.getParam("wheel_base", wheel_base);
    _nh.getParam("wheel_radius", wheel_radius);
    _nh.getParam("left_wheel_joint", _left_wheel_joint);
    _nh.getParam("right_wheel_joint", _right_wheel_joint);
    
    _robot = rigid2d::DiffDrive(wheel_base, wheel_radius);

    set_noises_and_errors();

    _cmd_vel_sub = _nh.subscribe("cmd_vel", 100, 
                                 &TubeWorldSimulator::update_robot_state,
                                 (rigid2d::DiffDriveSimulator*) this);

    _joints_pub = _nh.advertise<sensor_msgs::JointState>("joint_states", 100);
  }

  void run()
  {
    double frequency;
    _private_nh.getParam("frequency", frequency);
    
    auto rate = ros::Rate(frequency);
    while(ros::ok())
    {
      ros::spinOnce();
      auto joints_msg = get_joints_state_message(_left_wheel_joint,
                                                 _right_wheel_joint);
      _joints_pub.publish(joints_msg);
      rate.sleep();
    }
  }

private:
  rigid2d::DiffDriveSimulator _driver;
  std::string _left_wheel_joint, _right_wheel_joint;

  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  ros::Subscriber _cmd_vel_sub;
  ros::Publisher _joints_pub;

  std::default_random_engine _random_generator;
  
  sensor_msgs::JointState
  get_joints_state_message(std::string const& left_wheel_joint,
                           std::string const& right_wheel_joint)
  {
    auto wheels = _robot.wheelsPosition();
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

  void set_noises_and_errors()
  {
    double linear_vel_mu, linear_vel_sigma;
    _private_nh.getParam("linear_velocity_mu", linear_vel_mu);
    _private_nh.getParam("linear_velocity_sigma", linear_vel_sigma);

    std::normal_distribution<double> linear_vel_error_dist(linear_vel_mu,
                                                           linear_vel_sigma);

    _linear_velocity_error = std::bind(linear_vel_error_dist, 
                                       std::ref(_random_generator));

    double angular_vel_mu, angular_vel_sigma;
    _private_nh.getParam("angular_velocity_mu", angular_vel_mu);
    _private_nh.getParam("angular_velocity_sigma", angular_vel_sigma);

    std::normal_distribution<double> angular_vel_error_dist(angular_vel_mu,
                                                            angular_vel_sigma);

    _angular_velocity_error = std::bind(angular_vel_error_dist, 
                                        std::ref(_random_generator));

    double slip_min, slip_max;
    _private_nh.getParam("slipping_noise_min", slip_min);
    _private_nh.getParam("slipping_noise_max", slip_max);

     std::uniform_real_distribution<double> wheel_slipping_noise_dist(slip_min,
                                                                      slip_max);
    _wheel_slip_noise = std::bind(wheel_slipping_noise_dist, 
                                  std::ref(_random_generator));
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_turtle");
  
  TubeWorldSimulator simulation{};
  simulation.run();
}

                    
