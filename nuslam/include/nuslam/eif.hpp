#ifndef __EIF_SLAM_HPP // Extended Information Filter
#define __EIF_SLAM_HPP

#include "filter.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "nuslam/nuslam_system.hpp"
#include "rigid2d/rigid2d.hpp"
#include <math_utils/math.hpp>
#include <vector>
#include <tuple>

using math::vecd;
using math::matd;
using math::pointf;

namespace nuslam
{
constexpr int DiffDriveControlDim = 2;
class EIFSlamDiffDrive: public Filter<DiffDriveControlDim>
{
public:
  // robot state dim
  constexpr static int N = 3;
  // measurement dim
  constexpr static int J = 2;
  // control dim
  constexpr static int M = DiffDriveControlDim;
  
  const int map_size;

  EIFSlamDiffDrive(NuslamSystem& system,
                   vecd<N> const& robot_init_state,
                   matd<N> const& robot_init_covariance,
                   matd<N> process_noise_covariance,
                   matd<J> measurement_noise_covariance,
                   int map_size): 
    map_size{map_size}, 
    _landmarks_count{0},
    _system{system}
  {
    initInfoAndCovMatrix(robot_init_covariance);
    initStateAndInfoVector(robot_init_state);
    
    _process_noise_cov = process_noise_covariance;
    _measurement_noise_cov = measurement_noise_covariance;
    _measurement_noise_cov_inv = measurement_noise_covariance.inverse();
    
    _initialized_landmark = std::vector<bool>(map_size, false);
    _landmarks_lut = std::vector(map_size, -1);
  }
  
  
  vecd<> const& 
  update(std::vector<std::tuple<uint16_t, vecd<2>>> const& landmarks) override;
  
  vecd<> const& predict(vecd<M> const&) override;

  Eigen::Ref<const vecd<>> state() const override
  {
    return _state.head(N + J*_landmarks_count); 
  }

  matd<> poseCovariance() const override
  {
    return _cov_matrix.block(0,0,N,N);
  }

  rigid2d::DiffDrive const& robot() const override
  {
    return _system.robot();
  }
  
  matd<> landmarkCovariance(int landmark_index) const override
  {
    int index = N + J*landmark_index;
    return _cov_matrix.block<J,J>(index, index);
  }

private:
  std::vector<bool> _initialized_landmark;
  std::vector<int> _landmarks_lut;
  unsigned int _landmarks_count;
  NuslamSystem _system;
  matd<N> _process_noise_cov;
  matd<J> _measurement_noise_cov;
  matd<J> _measurement_noise_cov_inv;

  vecd<> _info_vector, _state;
  
  matd<> _info_matrix, _cov_matrix;
  
  void initStateAndInfoVector(vecd<N> robot_init_state);
  void initInfoAndCovMatrix(matd<N> const& robot_init_covariance);
  void predictCovMatrix(matd<N>&);
  void computeInfoMatAndVector(); 
  void initLandmark(uint16_t, vecd<2> const&);
  
};

} // end nuslam

#endif
