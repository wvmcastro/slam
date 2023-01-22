#include <ros/ros.h>
#include <nuslam/eif.hpp>

namespace nuslam
{
void
EIFSlamDiffDrive::initInfoAndCovMatrix(matd<N> const& robot_init_covariance)
{
  int size = N + J*map_size;
  _info_matrix = matd<>::Zero(size, size);

  _cov_matrix = matd<>::Zero(size, size);
  _cov_matrix.block<N,N>(0,0) = robot_init_covariance;
}

void
EIFSlamDiffDrive::initStateAndInfoVector(vecd<N> robot_init_state)
{
  int size = N + J*map_size;
  _state = vecd<>::Zero(size);
  _state.head<N>() = robot_init_state;
  
  _info_vector = vecd<>::Zero(size);
}

vecd<> const& 
EIFSlamDiffDrive::predict(vecd<M> const& wheels_position) 
{
  float old_phi = _state(0);

  auto pose = _system.robot().updateOdometry(wheels_position(0),
                                             wheels_position(1));
  _state(0) = rigid2d::normalize_angle(pose.theta);
  _state(1) = pose.x;
  _state(2) = pose.y;
  
  auto G = _system.robotJacobian(_state, old_phi);
  predictCovMatrix(G); 
 
  return _state;
}

void 
EIFSlamDiffDrive::predictCovMatrix(matd<N>& robot_model_jacobian)
{
  int n = N + _landmarks_count*J;
  

  matd<> G = matd<>::Identity(n, n);
  G.block<N, N>(0, 0) = robot_model_jacobian;

  _cov_matrix.block(0,0,n,n) = G*_cov_matrix.block(0,0,n,n)*G.transpose();
  _cov_matrix.block<N,N>(0,0) += _process_noise_cov;
}

vecd<> const& 
EIFSlamDiffDrive::update(
    std::vector<std::tuple<uint16_t, vecd<J>>> const& measurements_dict)
{
  if(_landmarks_count > 0)
  {
    // we do not need to compute the info matrix and vector
    // if there will be no correction. This only happens when the first 
    // measurements are added
    computeInfoMatAndVector();
  }

  std::vector<uint16_t> uninitialized_landmarks;
  uint16_t  i = 0;
  for(auto& el : measurements_dict)
  {
    int landmark_id = std::get<0>(el);

    if(_initialized_landmark[landmark_id] == true)
    {
      int n = N + J*_landmarks_count;

      // landmark's first component in the state vector
      landmark_id = _landmarks_lut[landmark_id]; 

      auto H = _system.lidarJacobian(_state.head(n), landmark_id);
      auto H_t = H.transpose();
      auto& Q_inv = _measurement_noise_cov_inv; 

      // info matrix update
      _info_matrix.block(0, 0, n, n) = _info_matrix.block(0, 0, n, n) + 
        H_t*Q_inv*H;
      
      // h(\mu_k)
      vecd<J> z = _system.lidarMeasurementPrediction(_state, landmark_id); 
      auto& y = std::get<1>(el);
      vecd<> error = y - z;
      error(1) = rigid2d::normalize_angle(error(1));

      // info vector update
      auto& xi = _info_vector;
      xi.head(n) = xi.head(n) + H_t*Q_inv * (error + H*_state.head(n)); 
      
      matd<> inv_info_mat = _info_matrix.block(0,0,n,n).inverse();
      _cov_matrix.block(0,0,n,n) = inv_info_mat;
      _state.head(n) = inv_info_mat * xi.head(n);
      _state(0) = rigid2d::normalize_angle(_state(0));
    }
    else
    {
      uninitialized_landmarks.push_back(i);
    }
    i++;
  }

  for(auto idx: uninitialized_landmarks)
  {
    auto id = std::get<0>(measurements_dict[idx]);
    auto& measurement = std::get<1>(measurements_dict[idx]);
    initLandmark(id, measurement);
  }

  // update robot drive state
  _system.robot().reset(_state(1), _state(2), _state(0)); 
  
  return _state;
}

void 
EIFSlamDiffDrive::initLandmark(uint16_t id, vecd<2> const& measurement)
{
  if(_landmarks_count < map_size)
  {
    _system.initLandmark(_state, _landmarks_count, measurement);

    int n = N + 2*_landmarks_count;

    // covariance 
    auto inv_model_jacobians = _system.invMeasurementModelJacobian(_state, 
      measurement);
    auto& Gr = inv_model_jacobians.first;
    auto& Gy = inv_model_jacobians.second;
    _cov_matrix.block<J,J>(n, n) = Gr*_cov_matrix.block<N,N>(0,0)*Gr.transpose() 
      + Gy*_measurement_noise_cov*Gy.transpose();
    _cov_matrix.block(n, 0, J, n) = Gr * _cov_matrix.block(0, 0, N, n); 
    _cov_matrix.block(0, n, n, J) = _cov_matrix.block(n, 0, J, n).transpose();
    
    _initialized_landmark[id] = true;
    _landmarks_lut[id] = _landmarks_count;
    _landmarks_count++;
  }
}

void
EIFSlamDiffDrive::computeInfoMatAndVector()
{
  int n = N + J*_landmarks_count;
  
  matd<> inv = _cov_matrix.block(0,0,n,n).inverse();
  _info_matrix.block(0,0,n,n) = inv;
  _info_vector.head(n) = inv*_state.head(n);
}

}
