#ifndef __FILTER_HPP
#define __FILTER_HPP

#include "rigid2d/diff_drive.hpp"
#include <math_utils/math.hpp>
#include <vector>
#include <tuple>

using math::vecd;
using math::matd;

namespace nuslam
{
template <int M>
class Filter
{
public:
  virtual vecd<> const& 
    update(std::vector<std::tuple<uint16_t, vecd<2>>> const& landmarks) = 0;
  
  virtual vecd<> const& predict(vecd<M> const&) = 0;

  virtual Eigen::Ref<const vecd<>> state() const = 0;

  virtual matd<> poseCovariance() const = 0;

  virtual rigid2d::DiffDrive const& robot() const = 0;

  virtual matd<> landmarkCovariance(int landmark_index) const = 0;
};

} // end nuslam

#endif
