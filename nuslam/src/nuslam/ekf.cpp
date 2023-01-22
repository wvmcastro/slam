#include <nuslam/ekf.hpp>
#include <ros/ros.h>

constexpr int LEFT = 0;
constexpr int RIGHT = 1;

using namespace nuslam;

template <typename T, int m, int n>
bool hasNaN(Eigen::Matrix<T, m, n> const& mat)
{
  for(int i = 0; i < mat.size(); i++)
  {
    if(std::isnan(mat(i)) == true)
      return true;
  }
  
  return false;
}

void 
EKFSlamDiffDrive::initStateVector(vecd<N> robot_init_state)
{
  _state = Eigen::VectorXd::Zero(N + 2*map_size);
  _state.head(N) = robot_init_state;
  _state(0) = rigid2d::normalize_angle(_state(0));
}

void
EKFSlamDiffDrive::initCovarianceMatrix(
    matd<N> const& robot_init_covariance,
    matd<J> const& measurement_noise_covariance)
{
  int n = N + J*map_size;
  _cov = matd<>::Zero(n, n);
  _cov.block<N, N>(0, 0) = robot_init_covariance;
}


vecd<> const&
EKFSlamDiffDrive::predict(vecd<M> const& wheels_position)
{
  float old_phi = _state(0);

  auto pose = _system.robot().updateOdometry(wheels_position(0),
                                             wheels_position(1));
  _state(0) = rigid2d::normalize_angle(pose.theta);
  _state(1) = pose.x;
  _state(2) = pose.y;
  
  auto Gr = _system.robotJacobian(_state, old_phi);
  predictCovariance(Gr); 
  
  return _state;
}

void
EKFSlamDiffDrive::predictCovariance(matd<N>& robot_model_jacobian)
{
  auto& Gr = robot_model_jacobian;

  // predict \Sigma_r
  _cov.block<N,N>(0,0) = Gr * _cov.block<N,N>(0,0) * Gr.transpose() 
    + _process_noise_cov;
  
  int k = J*_landmarks_count;
  
  // predict \Sigma_{RM}
  _cov.block(0, N, N, k) = Gr * _cov.block(0, N, N, k);
  // Sigma_{MR} = \Sigma_{RM}^T
  _cov.block(N, 0, k, N) = _cov.block(0, N, N, k).transpose();
}

vecd<> const& 
EKFSlamDiffDrive::update(
    std::vector<std::tuple<uint16_t, vecd<2>>> const& measurements_dict)
{
  uint16_t i = 0;
  std::vector<uint16_t> uninitialized_landmarks;
  for(auto& el : measurements_dict)
  {
    int id = std::get<0>(el);
    if(initialized_landmark[id] == true)
    {
      int n = N + 2*_landmarks_count; 

      id = _landmarks_lut[id];

      auto H = _system.lidarJacobian(_state.head(n), id);
      matd<> K = kalmanGain(H);
      
      vecd<J> z_pred = _system.lidarMeasurementPrediction(_state.head(n), id);
      
      vecd<2> measurement = std::get<1>(el);
      vecd<J> error = measurement - z_pred;
      
      error(1) = rigid2d::normalize_angle(error(1));
      _state.head(n) = _state.head(n) + K*error;
      _state(0) = rigid2d::normalize_angle(_state(0));
      
      auto& R = _measurement_noise_cov;
      
      matd<> I = matd<>::Identity(n, n);

      // Eq. 2.58 from Optimal and Robust Estimation: With an Introduction
      // to Stochastic Control Theory - Lewis, Frank L.
      // Seems more computationally stable than the classic 
      // _cov = (I - K*H) * _cov; It guarantees the positive 
      // semidefiniteness of _cov in the presence of roundoff error and is often
      // used for actual software implementation of the filter
      _cov.block(0, 0, n, n) = ((I - K*H) * _cov.block(0, 0, n, n) 
          * (I - K*H).transpose()) + K*R*K.transpose();
    }
    else
    {
      uninitialized_landmarks.push_back(i);
    }
    i++;
  }

  for(auto idx : uninitialized_landmarks)
  {
    int id = std::get<0>(measurements_dict[idx]);
    auto& measurement = std::get<1>(measurements_dict[idx]);
    initLandmark(id, measurement);
  }

  // resets the robot drive to the current estimated pose 
  _system.robot().reset(_state(1), _state(2), _state(0)); 
  
  return _state;
}

void
EKFSlamDiffDrive::initLandmark(uint16_t id, vecd<J> const& measurement)
{
  if(id < map_size)
  {
    _system.initLandmark(_state, _landmarks_count, measurement);
    
    int n = N + J*_landmarks_count;
    initNewLandmarkCovariance(n, measurement);

    initialized_landmark[id] = true;
    _landmarks_lut[id] = _landmarks_count;
    _landmarks_count++;
  }
}

matd<>
EKFSlamDiffDrive::kalmanGain(spmatd& measurementModelJacobian)
{
  // Looks like the kalman gain goes singular when setting the measurement
  // erro to 0. So a Covariance salt must be added:
  // https://robotics.stackexchange.com/questions/519/ekf-slam-update-step-kalman-gain-becomes-singular

  int n = N + 2*_landmarks_count;
  auto& H = measurementModelJacobian;
  auto H_t = H.transpose();
  
  matd<> K = _cov.block(0, 0, n, n)*H_t * (H*_cov.block(0, 0, n, n)*H_t 
      + _measurement_noise_cov).inverse();

  return K;
}

void
EKFSlamDiffDrive::initNewLandmarkCovariance(int n, 
                                            vecd<J> const& measurement)
{
  auto inv_model_jacobians = _system.invMeasurementModelJacobian(_state, 
      measurement);
  auto& Gr = inv_model_jacobians.first;
  auto& Gy = inv_model_jacobians.second;
  _cov.block<J,J>(n, n) = Gr*_cov.block<N,N>(0,0)*Gr.transpose() 
    + Gy*_measurement_noise_cov*Gy.transpose();
  _cov.block(n, 0, J, n) = Gr * _cov.block(0, 0, N, n); 
  _cov.block(0, n, n, J) = _cov.block(n, 0, J, n).transpose();
}
