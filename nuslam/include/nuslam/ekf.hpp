#ifndef __EKF_SLAM_HPP
#define __EKF_SLAM_HPP

#include "filter.hpp"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuslam_system.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math_utils/math.hpp>
#include <vector>
#include <tuple>

using math::vecd;
using math::matd;
using math::spmatd;
using math::pointf;

namespace nuslam
{
constexpr int DiffDriveControlDim = 2;
class EKFSlamDiffDrive: public Filter<DiffDriveControlDim>
{
public:
  // robot state dim
  constexpr static int N = 3;
  // measurement dim
  constexpr static int J = 2;
  // control dim
  constexpr static int M = DiffDriveControlDim;

  const int map_size;

  EKFSlamDiffDrive(NuslamSystem& system,
                   vecd<N> const& robot_init_state,
                   matd<N> const& robot_init_covariance,
                   matd<N> process_noise_covariace,
                   matd<J> measurement_noise_covariance,
                   int map_size): 
    map_size{map_size}, 
    _landmarks_count{0},
    _system{system}
  {
    initStateVector(robot_init_state);
    initCovarianceMatrix(robot_init_covariance, 
        measurement_noise_covariance);

    _process_noise_cov = process_noise_covariace;
    _measurement_noise_cov = measurement_noise_covariance;
    initialized_landmark = std::vector<bool>(map_size, false);
    _landmarks_lut = std::vector<int>(map_size, -1);
  }
  
  
  vecd<> const& 
  update(std::vector<std::tuple<uint16_t, vecd<2>>> const&) override;
  
  vecd<> const& predict(vecd<M> const&) override;

  Eigen::Ref<const vecd<>> state() const override
  {
    return _state.head(N + J*_landmarks_count); 
  }

  matd<> poseCovariance() const override
  {
    return _cov.block(0, 0, N, N);
  }

  rigid2d::DiffDrive const& robot() const override
  {
    return _system.robot();
  }

  matd<> landmarkCovariance(int landmark_index) const override
  {
    int index = N + J*landmark_index;
    return _cov.block<J, J>(index, index);
  }

private:
  std::vector<bool> initialized_landmark;
  std::vector<int> _landmarks_lut;
  unsigned int _landmarks_count;
  NuslamSystem _system;
  matd<N> _process_noise_cov;
  matd<J> _measurement_noise_cov;

  vecd<> _state;
  
  matd<> _cov;
  
  void initStateVector(vecd<N> robot_init_state);
  void initCovarianceMatrix(matd<N> const& robot_init_covariance,
                               matd<J> const& measurement_noise_covariance);
  void initLandmark(uint16_t, vecd<J> const&);
  void initNewLandmarkCovariance(int, vecd<J> const&);
  
  void predictCovariance(matd<N>&);

  matd<> kalmanGain(spmatd&);
};

} // end nuslam

#endif
