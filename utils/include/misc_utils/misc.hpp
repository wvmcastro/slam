#ifndef __MISC_HPP_
#define __MISC_HPP_

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <eigen3/Eigen/Dense>

namespace math
{
struct MyTransformData;

}

template <typename T>
void swap(T& a, T& b)
{
  T aux = a;
  a = b;
  b = aux;
}

template <typename T>
void swap(T& v)
{
  auto aux = v[0];
  v[0] = v[1];
  v[1] = aux;
}


template <typename T1, typename T2>
std::basic_ostream<char>& operator<<(std::basic_ostream<char> &ss, 
  std::pair<T1, T2> const &p) 
{
  ss << "(" << std::get<0>(p) << "," << std::get<1>(p) << ")";
  return ss;
}

template <typename T>
void correctIdsAfterRemoval(std::vector<T>& ids, std::vector<T> const& removed)
{
  int size = ids.size();
  
  int k = 0;
  int n = removed.size();
  for(int i = 0; i < size; i++)
  {
    T& id = ids[i];
    while(k < n && removed[k] < id) k++;

    ROS_INFO_STREAM("Updating " << id << "  to " << id-k);
    id = id - k;
  }
}

namespace misc
{

math::MyTransformData
getTransform(std::string const& destination_frame,
  std::string const& source_frame, tf2_ros::Buffer const& tf2_buffer,
  ros::Time time=ros::Time::now(),
  ros::Duration delay=ros::Duration(0));

Eigen::Matrix<double, 3, 3, Eigen::ColMajor>
getTransformMatrix(std::string const& destination_frame,
  std::string const& source_frame, tf2_ros::Buffer const& tf2_buffer,
  ros::Time time=ros::Time::now(),
  ros::Duration delay=ros::Duration(0));

}

#endif