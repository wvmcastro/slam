#include <misc_utils/misc.hpp>
#include <math_utils/math.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace misc
{

math::MyTransformData
getTransform(std::string const& destination_frame,
  std::string const& source_frame, tf2_ros::Buffer const& tf2_buffer,
  ros::Time time, ros::Duration delay)
{
  geometry_msgs::TransformStamped transform_msg = tf2_buffer.lookupTransform(
    destination_frame, source_frame, time);
  
  tf2::Quaternion quat;
  tf2::fromMsg(transform_msg.transform.rotation, quat);
  double yaw = math::quaternionToHeading<double>(quat);

  return math::MyTransformData{ transform_msg.transform.translation.x,
    transform_msg.transform.translation.y, yaw};
}

math::mat3d
getTransformMatrix(std::string const& destination_frame,
  std::string const& source_frame, tf2_ros::Buffer const& tf2_buffer,
  ros::Time time, ros::Duration delay)
{
  math::MyTransformData t = getTransform(destination_frame, source_frame, 
    tf2_buffer, time, delay);
    
  math::mat3d m = math::mat3d::Zero();

  m(0,2) = t.translation.x;
  m(1,2) = t.translation.y;

  m(0,0) = m(1,1) = std::cos(t.rotation);
  m(0,1) = std::sin(t.rotation);
  m(1,0) = -m(0,1);

  m(2,2) = 1;

  return m;
}

}