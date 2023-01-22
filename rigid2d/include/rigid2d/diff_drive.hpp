#ifndef DIFF_DRIVE_H_
#define DIFF_DRIVE_H_

#include <geometry_msgs/Twist.h>
#include "rigid2d/rigid2d.hpp"
#include <eigen3/Eigen/Dense>

namespace rigid2d
{


class WheelsVelocities
{
public:
  double left, right;
  WheelsVelocities(): left{0}, right{0} {}
  WheelsVelocities(double left_, double right_): 
    left{left_}, right{right_}
  {
    // do nothing
  }
};

class DiffDrive
{
public:
    /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
    DiffDrive(): _left_wheel_angle{0}, _right_wheel_angle{0},
      _wheels_velocities{}
    {
      _pose = {0, 0, 0};
      _wheel_base = 0;
      _wheel_radius = 0;
    }

    /// \brief create a DiffDrive model by specifying the pose, and geometry
    ///
    /// \param pose - the current position of the robot
    /// \param wheel_base - the distance between the wheel centers
    /// \param wheel_radius - the raidus of the wheels
    DiffDrive(struct Pose robotPose, double wheel_base_, double wheel_radius_):
      _wheel_base{wheel_base_}, _wheel_radius{wheel_radius_}, 
      _left_wheel_angle{0}, _right_wheel_angle{0}
    {
        _pose = robotPose;
    }
    
    DiffDrive(double wheel_base_, double wheel_radius_):
      _wheel_base{wheel_base_}, _wheel_radius{wheel_radius_}, 
      _left_wheel_angle{0}, _right_wheel_angle{0}
    {
      _pose = {0, 0, 0};
    }
    /// \brief determine the wheel velocities required to make the robot
    ///        move with the desired linear and angular velocities
    /// \param twist - the desired twist in the body frame of the robot
    /// \returns - the wheel velocities to use
    /// \throws std::exception
    WheelsVelocities twistToWheels(Twist const& twist_in_body_frame) const;

    /// \brief determine the body twist of the robot from its wheel velocities
    /// \param vel - the velocities of the wheels, assumed to be held constant
    ///  for one time unit
    /// \returns twist in the original body frame of the
    Twist wheelsToTwist(WheelsVelocities const& vel) const;

    /// \brief Update the robot's odometry based on the 
    ///        current encoder readings
    /// \param left - the left encoder angle (in radians)
    /// \param right - the right encoder angle (in radians)
    struct Pose const&
    updateOdometry(double left_encoder, double right_encoder);

    /// \brief update the odometry of the diff drive robot, assuming that
    /// it follows the given body twist for one time  unit
    /// \param cmd - the twist command to send to the robot
    void feedforward(Twist const& tw, bool update_wheels=true);

    /// \brief get the current pose of the robot
    Pose const& pose() const
    {
      return _pose;
    }

    
    /// \brief get the wheel speeds, based on the last encoder update
    /// \returns the velocity of the wheels, which is equivalent to
    /// displacement because \Delta T = 1
    WheelsVelocities const& wheelsVelocities() const
    {
      return _wheels_velocities;
    }

    Vector2D wheelsPosition()
    {
      return Vector2D(_left_wheel_angle, _right_wheel_angle);
    }

    /// \brief reset the robot to the given position/orientation
    void reset(double x, double y, double theta);

    double wheelBase() const
    {
      return _wheel_base;
    }

    double wheelRadius() const
    {
      return _wheel_radius;
    }

    Eigen::Vector3d const& computeMotionDelta(double left_encoder, 
                                              double right_encoder) const;

private:
  double _wheel_base, _wheel_radius;
  double _left_wheel_angle, _right_wheel_angle;
  struct Pose _pose;
  WheelsVelocities _wheels_velocities;

  friend class DiffDriveSimulator;
};

class DiffDriveSimulator
{
public:
    
  DiffDriveSimulator()
  {
    // do nothing
  }
  
protected:
  DiffDrive _robot;
  std::function<double()> _linear_velocity_error, _angular_velocity_error;
  std::function<double()> _wheel_slip_noise;
  
  void update_robot_state(geometry_msgs::Twist::ConstPtr const& twist)
  {
    rigid2d::Twist tw(twist->angular.z, twist->linear.x, twist->linear.y);
    
    // Add noise to the command
    tw.w += _angular_velocity_error();
    tw.v.x += _linear_velocity_error(); 
    _robot.feedforward(tw);

    auto n = _wheel_slip_noise();
    _robot._left_wheel_angle += n * _robot._wheels_velocities.left; 
    _robot._right_wheel_angle += n * _robot._wheels_velocities.right; 
  }

};

}
#endif
