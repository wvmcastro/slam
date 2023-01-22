#include "rigid2d/diff_drive.hpp"
#include <stdexcept>
#include <ros/ros.h>

namespace rigid2d
{
  Twist
  DiffDrive::wheelsToTwist(WheelsVelocities const& vel) const
  {
    auto ur = vel.right;
    auto ul = vel.left;
    double vx = 0.5 * _wheel_radius * (ur + ul);
    double theta_dot = (_wheel_radius /  _wheel_base) * (ur - ul);

    return Twist(theta_dot, vx, 0);
  }

  WheelsVelocities 
  DiffDrive::twistToWheels(Twist const& twist_in_body_frame) const
  {
    auto v = twist_in_body_frame.v;

    if(v.y != 0)
    {
      throw std::invalid_argument("Twist cannot have speed in the body frame"
                                  " axis, because of the constraints of the"
                                  " differential drive robot");
    }

    auto theta_dot = twist_in_body_frame.w;
    auto inv_r = 1.0 / _wheel_radius;
    auto d = 0.5 * _wheel_base;
    
    auto ur = inv_r * (v.x + d*theta_dot);
    auto ul = inv_r * (v.x - d*theta_dot);
    return WheelsVelocities(ul, ur);
  }

  void
  DiffDrive::reset(double x, double y, double theta)
  {
    _pose.x = x;
    _pose.y = y;
    _pose.theta = theta;
    _wheels_velocities = WheelsVelocities();
  }

  
  struct Pose const&
  DiffDrive::updateOdometry(double left_encoder, double right_encoder)
  {
    auto& wv = _wheels_velocities;
    wv.left = normalize_angle(left_encoder - _left_wheel_angle); 
    wv.right = normalize_angle(right_encoder - _right_wheel_angle);

    _left_wheel_angle = left_encoder;
    _right_wheel_angle = right_encoder;

    auto tw = wheelsToTwist(_wheels_velocities);

    feedforward(tw, false);
    
    return _pose;
  }

  void
  DiffDrive::feedforward(Twist const& tw, bool update_wheels)
  {
    auto tr = Transform2D(Vector2D(_pose.x, _pose.y), _pose.theta);
    auto tr2 = tr.integrateTwistForOneTimeUnit(tw);

    auto v = twistToWheels(tw);
    if(update_wheels == true)
    {
      _left_wheel_angle += v.left;
      _right_wheel_angle += v.right;
      
      auto& wv = _wheels_velocities;
      wv.left = v.left; 
      wv.right = v.right;
    }

    _pose.theta = tr2.theta();
    _pose.x = tr2.x();
    _pose.y = tr2.y();
  }
  
  Eigen::Vector3d const& 
  DiffDrive::computeMotionDelta(double left_encoder, double right_encoder) const
  {
    static Eigen::Vector3d delta_pose;
    using std::cos, std::sin;

    double theta_l = normalize_angle(left_encoder - _left_wheel_angle); 
    double theta_r = normalize_angle(right_encoder - _right_wheel_angle);

    if(almost_equal(theta_l, theta_r, 1e-6))
    {
      delta_pose(0) = 0;
      delta_pose(1) = theta_l*_wheel_radius * cos(_pose.theta);
      delta_pose(2) = theta_l*_wheel_radius * sin(_pose.theta);
    }
    else // there is angular velocity
    {
      double a = _wheel_radius / _wheel_base; 
      delta_pose(0) = normalize_angle(a*(theta_r - theta_l));

      double b = (0.5 * _wheel_base) * (theta_l + theta_r) / (theta_r - theta_l);
      delta_pose(1) = b * (sin(_pose.theta + delta_pose(0)) - sin(_pose.theta));
      delta_pose(2) = b * (-cos(_pose.theta + delta_pose(0)) + cos(_pose.theta));
    }
    return delta_pose;
  }
    
}

