#ifndef __FEATURE_DESCRIPTOR_HPP_
#define __FEATURE_DESCRIPTOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <array>

namespace pointcloud
{

template <int N>
class FeatureDescriptor
{
public:
  FeatureDescriptor() :
    _descriptor{}
  {
    // do nothing
  }

  // -pi < theta < pi
  void insert(double theta)
  {
    int index = theta / _resolution;
    
    if(theta >= 0)
      _descriptor[index] += 1;
    else
    {
      int correctedIndex = N + index - 1;
      _descriptor[correctedIndex] += 1;
    }
  }

  FeatureDescriptor operator-(FeatureDescriptor const& other) const
  {
    FeatureDescriptor result;
    for(int i = 0; i < N; i++)
      result._descriptor[i] = this->_descriptor[i] - other._descriptor[i];

    return result;
  }

  double squared() const
  {
    double squaredNorm = 0;
    for(int i = 0; i < N; i++)
      squaredNorm += std::pow(_descriptor[i], 2);
    
    return squaredNorm;
  }

  std::array<int, N> const& get() const
  {
    return _descriptor;
  }

  unsigned int size() const
  {
    return N;
  }

private:
  std::array<int, N> _descriptor;
  constexpr static double _resolution{M_PI / (N/2.0)};
};

}

#endif