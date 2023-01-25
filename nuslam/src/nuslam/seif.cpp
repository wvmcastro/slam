#include <nuslam/seif.hpp>
#include <algorithm>
#include <eigen3/Eigen/SparseLU>
#include <misc_utils/misc.hpp>
#include <ros/ros.h>

namespace nuslam
{
void
SEIFSlamDiffDrive::initInfoMatrix(matd<N> const& robot_init_info)
{
  int size = N + J*map_size;
  _info_matrix = spmatd(size, size);
  auto reserve_size = std::vector<int>(size,2*N);
  _info_matrix.reserve(reserve_size);
  math::unsafeFillSparseWithDenseMatrix(_info_matrix, robot_init_info);
}

void
SEIFSlamDiffDrive::initStateAndInfoVector(vecd<N> robot_init_state)
{
  int size = N + J*map_size;
  _state = vecd<>::Zero(size);
  _state.head<N>() = robot_init_state;
  
  _info_vector = vecd<>::Zero(size);
}

vecd<> const& 
SEIFSlamDiffDrive::predict(vecd<M> const& wheels_position) 
{
  ROS_INFO_STREAM("Prediction step");
  // this function will compute new info vector \bar{\xi}, 
  // info matrix \bar{\Omega_t} and, new state vector \bar{\mu}

  // motion delta
  auto motion_delta = _system.robot().computeMotionDelta(wheels_position(0),
                                                         wheels_position(1));
  // Jacobian with respect to he robot. Complete Jacobian is
  // |F_r 0| 
  // |0   I|  
  vecd<1> new_heading{ _state(0) + motion_delta(0) };
  auto Fr = _system.robotJacobian(new_heading, _state(0));
  seif::MotionUpdateVariables motion_variables = getMotionUpdateVariables(Fr);
  
  // info matrix prediction, lambda was already 
  // subtracted in the computePhi step
  _info_matrix -= motion_variables.Kappa;

  predictInfoVector(motion_delta, motion_variables);

  _state.head<N>() += motion_delta;
  normalize_heading();
  _system.robot().updateOdometry(wheels_position(0), wheels_position(1));
  return _state;
}

seif::MotionUpdateVariables 
SEIFSlamDiffDrive::getMotionUpdateVariables(matd<N> const & robot_model_jacobian)
{
  seif::MotionUpdateVariables variables;
  variables.Psi = computePsi(robot_model_jacobian);
  variables.Lambda = computeLambda(variables.Psi);
  _info_matrix += variables.Lambda; // computePhi
  variables.Kappa = computeKappa(_info_matrix);

  return variables;
}

spmatd
SEIFSlamDiffDrive::computePsi(matd<N> const& robot_model_jacobian) const
{
  auto& Fr = robot_model_jacobian;
  auto Psi_r = Fr.inverse() - matd<3>::Identity();

  int n = _info_matrix.rows(); 
  auto Psi = spmatd(n, n);
  Psi.insert(1,0) = Psi_r(1,0); // x, phi
  Psi.insert(2,0) = Psi_r(2,0); // y, phi 

  return Psi;
}

spmatd
SEIFSlamDiffDrive::computeLambda(spmatd const& Psi) const
{
  int n = _info_matrix.rows(); 
  spmatd Lambda(n,n);
  auto& Omega = _info_matrix;
  spmatd Psi_t = Psi.transpose();
  Lambda = Omega*Psi + Psi_t*Omega + Psi_t*Omega*Psi;
  return Lambda;
}

spmatd
SEIFSlamDiffDrive::computeKappa(spmatd const& Phi) const
{
  auto& R_inv = _process_noise_cov_inv;
  matd<N> aux = (R_inv + Phi.block(0,0, N, N).toDense()).inverse();

  int n = _info_matrix.rows(); 
  spmatd aux2(n,n);
  for(int j = 0; j < N; j++) 
    for(int i = 0; i < N; i++)
      aux2.insert(i,j) = aux(i,j);
  
  spmatd Kappa(n,n);
  Kappa = Phi * aux2 * Phi;
  return Kappa;
}

void 
SEIFSlamDiffDrive::predictInfoVector(vecd<N> const& motion_delta, 
  seif::MotionUpdateVariables const & motion_variables)
{
  auto& v  = motion_variables;
  _info_vector += (v.Lambda - v.Kappa)*_state
    + _info_matrix.block(0, 0, _info_matrix.rows(), N)*motion_delta;
}

void 
SEIFSlamDiffDrive::normalize_heading()
{
  double normalized_heading = rigid2d::normalize_angle(_state(0));
  for(spmatd::InnerIterator it(_info_matrix, 0); it; ++it)
  {
    _info_vector(it.row()) += it.value() * (normalized_heading - _state(0));
  }
  _state(0) = normalized_heading;
}

vecd<> const& 
SEIFSlamDiffDrive::update(
    std::vector<std::tuple<uint16_t, vecd<J>>> const& measurements_dict)
{
  ROS_INFO_STREAM("Update step");
  ROS_INFO_STREAM("Number of measurements: " << measurements_dict.size());
  std::vector<vecd<J>> measurements;
  for(auto& p : measurements_dict)
    measurements.push_back(std::get<1>(p));

  auto r = dataAssociation(measurements);
  auto& associations = r.first;
  ROS_INFO_STREAM("Associations: " << associations);

  // update landmarks manager
  std::vector<uint16_t> candidates_to_remove;
  {
    std::vector<uint16_t> revisited_landmarks(associations.size());
    for(int i = 0; i < associations.size(); i++)
      revisited_landmarks[i] = associations[i].second;
    candidates_to_remove = _all_landmarks_manager.update(revisited_landmarks);
  }
  
  auto measured_but_inactive = _update(measurements, associations);

  ROS_INFO_STREAM("Measured but inactive: " << measured_but_inactive);

  auto active_to_passive_landmarks = _active_landmarks_manager.insert(
    measured_but_inactive.data(), measured_but_inactive.size());
  
  sparsify(active_to_passive_landmarks);

  _all_landmarks_manager.removeCandidates(candidates_to_remove);
  reduceInfoMatrixAndVectors(candidates_to_remove);
  
  auto uninitialized_landmarks = std::get<1>(r);
  for(auto idx: uninitialized_landmarks)
  {
    uint16_t id = initLandmark(measurements[idx]);
    if(id != std::numeric_limits<uint16_t>::max())
      _all_landmarks_manager.insertCandidate(id);
  }
  
  // update robot drive state
  _system.robot().reset(_state(1), _state(2), _state(0));

  return _state;
}

std::vector<uint16_t>
SEIFSlamDiffDrive::_update(std::vector<vecd<2>> const& measurements, 
  std::vector<std::pair<uint16_t, uint16_t>> const& associations)
{
  ROS_INFO_STREAM("Known association update step");
  std::vector<uint16_t> measured_but_inactive(0);
  for(auto& pair : associations)
  {
    uint16_t landmark_id = std::get<1>(pair);

    spmatd H;
    bool landmark_is_permanent = _all_landmarks_manager.isPermanent(landmark_id);
    if(landmark_is_permanent)
    {
      auto r = _active_landmarks_manager.update(landmark_id);
      if(r == false)
        measured_but_inactive.push_back(landmark_id);

      H = _system.lidarJacobian(_state, landmark_id, true);
    }
    else
      H = _system.lidarJacobian(_state, landmark_id, false);
    
    auto H_t = H.transpose();
    auto& Q_inv = _measurement_noise_cov_inv; 

    // info matrix update
    _info_matrix = (_info_matrix + H_t*Q_inv*H).pruned();

    // h(\mu_k)
    vecd<J> z = _system.lidarMeasurementPrediction(_state, landmark_id); 

    uint16_t measurement_index = std::get<0>(pair);
    auto& y = measurements[measurement_index];
    vecd<> error = y - z;
    error(1) = rigid2d::normalize_angle(error(1));

    // info vector update
    auto& xi = _info_vector;
    xi = xi + H_t*Q_inv * (error + H*_state); 

    if(landmark_is_permanent) 
      updateMeanVector(std::vector<uint16_t>(1, landmark_id));
  }

  return measured_but_inactive;
}

// TODO: this is a very lazy method that must be refactored after the seif poc
void 
SEIFSlamDiffDrive::updateMeanVector(
  std::vector<uint16_t> const& landmarks_index,
  bool update_robot_state,
  int max_iterations)
{
  auto& omega = _info_matrix;
  auto& xi = _info_vector;

  for(int k = 0; k < max_iterations; k++)
  {
    // landmarks
    for(auto landmark_index : landmarks_index)
    {
      spmatd F = makeProjectionMatrix(
        std::vector<uint16_t>(1, landmark_index)).cast<double>();
      int l = N + J*landmark_index;
      _state.segment<J>(l) = (F*omega*F.transpose()).toDense().inverse() * 
        F*(xi - omega*_state + omega*F.transpose()*F*_state);
    }
    
    // robot state
    if(update_robot_state == true)
    {
      spmatd F = makeProjectionMatrix({}, true).cast<double>();
      _state.head<N>() = (F*omega*F.transpose()).toDense().inverse() * 
        F*(xi - omega*_state + omega*F.transpose()*F*_state);
    }
  }
  normalize_heading();
}

uint16_t 
SEIFSlamDiffDrive::initLandmark(vecd<2> const& measurement)
{
  if(_landmarks_count < map_size)
  {
    _system.initLandmark(_state, _landmarks_count, measurement);

    auto inv_model_jacobians = _system.invMeasurementModelJacobian(_state, 
      measurement, false);
    initLandmarkInInfoMat(inv_model_jacobians);
    initlandmarkInInfoVector();

    uint16_t filter_id = _landmarks_count;
    _landmarks_count++;

    return filter_id;
  }
  ROS_ERROR_STREAM("Init landmark error, map is full! Map size: " << map_size);
  return std::numeric_limits<uint16_t>::max();
}

void
SEIFSlamDiffDrive::initLandmarkInInfoMat(
  std::pair<matd<J,N>, matd<J>> const& inv_measurement_model_jacobians)
{
  auto& Gr = inv_measurement_model_jacobians.first;
  auto& Gz = inv_measurement_model_jacobians.second;

  auto Gz_inv = Gz.inverse();
  auto info_mm = Gz_inv.transpose()*_measurement_noise_cov_inv*Gz_inv;
  int n = N + J*_landmarks_count;
  math::unsafeFillSparseWithDenseMatrix(_info_matrix, info_mm, n, n);
  
  auto info_lr = -info_mm * Gr;
  math::unsafeFillSparseWithDenseMatrix(_info_matrix, info_lr, n, 0);
  math::unsafeFillSparseWithDenseMatrix(_info_matrix, info_lr.transpose(), 0, n);

  auto delta_info_rr = - Gr.transpose() * info_lr;
  for(int j = 0; j < N; j++)
  {
    for(spmatd::InnerIterator it(_info_matrix, j); it.row() < N; ++it)
    {
      it.valueRef() += delta_info_rr(it.row(),j);
    }
  }

  _info_matrix.makeCompressed();
}

void
SEIFSlamDiffDrive::initlandmarkInInfoVector()
{
  int n = N + J*_landmarks_count;
  _info_vector.head(N) = _info_matrix.block(0,0, N,n+J) * _state.head(n+J);
  _info_vector.segment(n, J) = _info_matrix.block(n,0, J,n+J)*_state.head(n+J);
}

void
SEIFSlamDiffDrive::sparsify(
  std::vector<uint16_t> const& active_to_passive_landmarks)
{
  if(active_to_passive_landmarks.size() == 0)
  {
    ROS_INFO_STREAM("There is no landmark to deactivate");
    return;
  }

  auto sparse_info_matrix = getSparseInfoMatrix(_info_matrix,
    _active_landmarks_manager.landmarks(), active_to_passive_landmarks);

  sparse_info_matrix.prune(1,1e-6);

  sparsifyVector(_info_vector, _state, _info_matrix, sparse_info_matrix);
  _info_matrix = sparse_info_matrix;
}

spmatd 
SEIFSlamDiffDrive::getSparseInfoMatrix(
  spmatd const& info_matrix,
  std::vector<uint16_t> const& active_landmarks,
  std::vector<uint16_t> const& active_to_passive_landmarks)
{
  uint16_t dim = info_matrix.rows();
  spmatd proj0;
  {
    std::vector<uint16_t> landmarks(active_to_passive_landmarks.begin(),
      active_to_passive_landmarks.end());
    landmarks.insert(landmarks.end(), 
      active_landmarks.begin(), active_landmarks.end());
    proj0 = seif::makeProjectionMatrix<N,J>(dim, landmarks, true).cast<double>();
  }
  auto proj0_t = proj0.transpose();
  spmatd omega0 = proj0_t * proj0*info_matrix*proj0_t * proj0;

  spmatb proj1 = seif::makeProjectionMatrix<N,J>(dim,
    active_to_passive_landmarks, false);
  spmatd omega1 = math::marginalizeInformationMatrix(omega0, proj1, epsilon);

  spmatb proj2 = seif::makeProjectionMatrix<N,J>(dim,
    active_to_passive_landmarks, true);
  spmatd omega2 = math::marginalizeInformationMatrix(omega0, proj2, epsilon);

  spmatb proj3 = seif::makeProjectionMatrix<N,J>(dim, {}, true);
  spmatd omega3 = math::marginalizeInformationMatrix(info_matrix, proj3, 
    epsilon);
  
  return  omega1 - omega2 + omega3;
}

void 
SEIFSlamDiffDrive::sparsifyVector(vecd<>& info_vector, vecd<> const& state,
  spmatd const& info_matrix, spmatd const& sparse_info_matrix)
{
  info_vector += (sparse_info_matrix - info_matrix) * state;
}

spmatb 
SEIFSlamDiffDrive::makeProjectionMatrix(
  std::vector<uint16_t> const& landmarks_indexes,
  bool robot) const
{
  return seif::makeProjectionMatrix<N,J>(N+J*map_size, landmarks_indexes, robot);
}

std::pair<std::vector<std::pair<uint16_t, uint16_t>>, std::vector<uint16_t> >
SEIFSlamDiffDrive::dataAssociation(std::vector<vecd<J>> const& measurements,
  bool only_in_neighborhood) const
{
  std::vector<std::pair<uint16_t, uint16_t>> associations;
  std::vector<uint16_t> not_associated;

  std::vector<uint16_t> neighborhood;
  if(only_in_neighborhood == true) 
    neighborhood = getNeighborhoodMap();
  else
    for(int i = 0; i < _landmarks_count; i++)
      neighborhood.push_back(i);

  std::vector<matd<J>> inv_cov_buffer = getDataAssociationInvCovariances(
    neighborhood);
  std::vector<vecd<J>> measurement_predictions = getMeasurementPredictions(
    neighborhood); 
  
  for(uint16_t j = 0; j < measurements.size(); j++)
  {
    double min_d = std::numeric_limits<double>::infinity();
    uint16_t nearest = -1; // max
    
    int k = 0;
    for(uint16_t i : neighborhood) 
    {
      vecd<J> diff = measurements[j] - measurement_predictions[k];
      diff(1) = rigid2d::normalize_angle(diff(1));
      double d = diff.transpose() * inv_cov_buffer[k] * diff;
      
      if(d < min_d)
      {
        min_d = d;
        nearest = i;
      }

      k++;
    }

    // validation gate
    if(min_d < data_association_chi_threshold)
      associations.push_back({j, nearest});
    else
      not_associated.push_back(j);
  }
  return {associations, not_associated};
}

std::vector<uint16_t> 
SEIFSlamDiffDrive::getNeighborhoodMap() const
{
  std::vector<uint16_t> neighborhood;  
  double cond = _info_matrix.block(0,0,N,N).toDense().inverse().norm();
  constexpr double sensor_range = 2;
  double distance_threshold = sensor_range + cond;
  for(int i = 0; i < _landmarks_count; i++)
  {
    double d = math::distance_l2<2>(_state.segment<2>(1), 
      _state.segment<2>(N+J*i));
    
    if(d <= distance_threshold)
      neighborhood.push_back(i);

  }

  return neighborhood;
}

std::vector<matd<SEIFSlamDiffDrive::J>>
SEIFSlamDiffDrive::getDataAssociationInvCovariances(
  std::vector<uint16_t> const& landmark_indexes) const
{
  std::vector<matd<J>> covariances;

  std::vector<uint16_t> active {_active_landmarks_manager.landmarks()};
  std::sort(active.begin(), active.end());
  for(uint16_t i : landmark_indexes)
  {
    uint16_t landmark_index_in_blanket;
    auto blanket = getDataAssociationMarkovBlanket(
      i, active, landmark_index_in_blanket);
    
    spmatd p1 = makeProjectionMatrix(blanket, true).cast<double>();
    spmatd p2 = seif::makeProjectionMatrix<N,J>(N+J*blanket.size(),
      std::vector(1, landmark_index_in_blanket), true).cast<double>();

    vecd<5> mean;
    mean.head<N>() = _state.head<N>();
    mean.segment<J>(N) = _state.segment<J>(N+J*i);

    matd<> info_blanket = (p1 * _info_matrix * p1.transpose());
    
    spmatd jacobian = _system.lidarJacobian(mean, 0,
      _all_landmarks_manager.isPermanent(i));
    
    matd<J> cov = jacobian * 
      (p2* info_blanket.inverse()*p2.transpose())
        * jacobian.transpose() + _measurement_noise_cov;

    covariances.push_back(cov.inverse());
  }

  return covariances;
}

std::vector<uint16_t> 
SEIFSlamDiffDrive::getDataAssociationMarkovBlanket(
  uint16_t landmark_id,
  std::vector<uint16_t> const& active_landmarks,
  uint16_t& landmark_index_in_blanket) const
{
  if(_all_landmarks_manager.isPermanent(landmark_id) == false)
  {
    landmark_index_in_blanket = 0;
    return std::vector<uint16_t>(1, landmark_id);
  }

  std::vector<uint16_t> neighbors = getNeighbors(
    landmark_id, 
    _info_matrix.col(N+J*landmark_id).toDense().data()+N, 
    _landmarks_count);

  bool intersect{false};
  std::vector<uint16_t> blanket = getSetsUnion(neighbors, 
                                               active_landmarks,
                                               intersect);
  if(intersect == false)
  {
    ROS_INFO_STREAM("There is no intersection between the "
                    "neighborhood of the target landmark and the "
                    "active landmarks set.");
    ROS_INFO_STREAM("Augmenting the blanket with BFS from landmark "
                    "to the robot");
    seif::GraphView<N,J> graph{_info_matrix, _landmarks_count};
    auto path = seif::shortestPathToRobot(graph, landmark_id);
    std::sort(path.begin(), path.end());

    blanket = getSetsUnion(blanket, path, intersect);
  }

  landmark_index_in_blanket = blanket.size(); // out of index range
  for(int k = 0; k < blanket.size(); k++)
    if(blanket[k] == landmark_id)
    {
      landmark_index_in_blanket = k;
      break;
    }
  
  if(landmark_index_in_blanket == blanket.size())
    blanket.push_back(landmark_id);

  return blanket;
}

std::vector<vecd<SEIFSlamDiffDrive::J>>
SEIFSlamDiffDrive::getMeasurementPredictions(
  std::vector<uint16_t> const& landmark_indexes) const
{
  std::vector<vecd<J>> predictions;
  
  for(uint16_t i : landmark_indexes)
    predictions.push_back(_system.lidarMeasurementPrediction(_state, i));

  return predictions;
}

void
SEIFSlamDiffDrive::transformInfoData(seif::Data& data,
  math::MyTransformData const& tr)
{
  int num_landmarks = data.state.size() / J;
  auto rotation_mat = math::rotationMatrix2D(tr.rotation);
  spmatd augmented_rotation_mat = math::constantBlockDiagonalMatrix(
    num_landmarks, rotation_mat);

  data.info_matrix = augmented_rotation_mat * data.info_matrix
    * augmented_rotation_mat.transpose();

  vecd<> delta(data.info_vector.size());
  vecd<2> translation{ tr.translation.x, tr.translation.y };
  for(int i = 0; i < num_landmarks; i++)
    delta.segment<J>(i*J) = translation;

  // THIS IS DIFFERENT FROM PROBABILIST ROBOTS
  // CHECK !!!
  data.info_vector = augmented_rotation_mat*data.info_vector + data.info_matrix*delta;
}

void 
SEIFSlamDiffDrive::integrateMap(seif::Data const& data)
{
  ROS_ERROR_STREAM("Current filter state size is " << N+J*_landmarks_count);
  // get measurements predictions
  vecd<N+J> fake_state;
  fake_state.head<N>() = _state.head<N>();

  int n = data.state.size() / J;
  std::vector<vecd<J>> fake_measurements(n);
  for(int i = 0; i < n; i++)
  {
    fake_state.tail<J>() = data.state.segment<J>(i*J);
    fake_measurements[i] = _system.lidarMeasurementPrediction(fake_state, 0);
  }
  auto r = dataAssociation(fake_measurements, false);

  ROS_INFO_STREAM("Found " << r.first.size() << " landmarks in common, and "
    << r.second.size() << " new.");

  vecd<> original_state = _state;
  ROS_INFO_STREAM("original state: "
    << original_state.head(internalStateSize()).transpose());
  
  // info augmentation
  auto augmented_filter_data = seif::mapInfoMerge(*this, data, r.first);

  // state (ie. mean) augmentation
  int augmented_landmarks_count = _landmarks_count;
  augmented_filter_data.state = _state;
  for(uint16_t index : r.second)
  {
    if(augmented_landmarks_count >= map_size)
      break;

    _system.initLandmark(augmented_filter_data.state,
      augmented_landmarks_count, fake_measurements[index]);

    _all_landmarks_manager.insertPermanent(augmented_landmarks_count++);
  }

  int augmented_dim = N + J*augmented_landmarks_count;
  
  _state = augmented_filter_data.state;
  
  ROS_INFO_STREAM("The augmented filter state size is: " 
    << augmented_dim);

  _info_vector.head(augmented_dim) = augmented_filter_data.info_vector;

  int filter_max_dim = N + J*map_size;
  augmented_filter_data.info_matrix.conservativeResize(filter_max_dim,
    filter_max_dim);

  ROS_INFO_STREAM("Augmented info matrix size: "
    << augmented_filter_data.info_matrix.rows() << "x"
    << augmented_filter_data.info_matrix.cols());
  
  ROS_INFO_STREAM("Filter info matrix size: " << _info_matrix.rows()
    << "x" << _info_matrix.cols());
  
  _info_matrix = augmented_filter_data.info_matrix;
  _landmarks_count = augmented_landmarks_count;

  for(auto& association : r.first)
  {
    uint16_t landmark_id = association.second;
    if(_all_landmarks_manager.isPermanent(landmark_id) == false)
      _all_landmarks_manager.upgradeCandidateToPermanent(landmark_id);

    bool update_robot_state = _active_landmarks_manager.isActive(landmark_id);
    ROS_INFO_STREAM("Update mean vector, landmark id: " << landmark_id <<
      ", update robot state: " << update_robot_state);
    updateMeanVector({1, landmark_id}, update_robot_state);
  }
  
  ROS_INFO_STREAM("augmented state: "
    << _state.head(augmented_dim).transpose());
  
  vecd<> diff = _state - original_state; 
  ROS_INFO_STREAM("states difference: " <<
    diff.head(augmented_dim).transpose());

}

void
SEIFSlamDiffDrive::reduceInfoMatrixAndVectors(std::vector<uint16_t> const& 
  landmarks_to_remove)
{
  if(landmarks_to_remove.size() == 0)
    return;

  ROS_INFO_STREAM("landmarks to remove: " << landmarks_to_remove);

  std::vector<uint16_t> active_landmarks = _active_landmarks_manager.landmarks();
  std::sort(active_landmarks.begin(), active_landmarks.end());

  std::vector<uint16_t> active_landmarks_new_ids = active_landmarks;
  correctIdsAfterRemoval(active_landmarks_new_ids, landmarks_to_remove);

  for(int i = 0; i < active_landmarks.size(); i++)
    _active_landmarks_manager.changeLandmarkId(active_landmarks[i],
      active_landmarks_new_ids[i]);

  std::vector<uint16_t> indexes_to_reduce(J*landmarks_to_remove.size());
  int i = 0;
  for(uint16_t landmark_id : landmarks_to_remove)
  {
    uint16_t index = _system.landmarkIdToVectorStateIndex(landmark_id);
    indexes_to_reduce[i++] = index;
    indexes_to_reduce[i++] = index + 1;
  }
  spmatd M = math::removeMatrix(N+J*map_size, indexes_to_reduce).cast<double>();
  spmatd M_t = M.transpose();

  _info_vector = M_t * _info_vector;
  _state = M_t * _state;
  _info_matrix = (M_t * _info_matrix * M).pruned();
  
  _landmarks_count -= landmarks_to_remove.size();
}

}

// If I include this header file at the top, the code stop compiling
// If you know why, please let me know. It will make me happy, even if years
// have been passed
#include <nuslam/MapExchangeSrv.h>

namespace nuslam
{
MapExchangeSrv::Response
SEIFSlamDiffDrive::getExchangeMapData() const
{
  MapExchangeSrv::Response res{};

  res.data.stamp = ros::Time::now();
  int dim = J*landmarks_count();
  res.data.system_dim = dim;
  auto state_vec = state();
  res.data.mean_vector = std::vector<double>(state_vec.data()+N,
                                             state_vec.data()+dim+N);
  spmatd info_mat = info_matrix();
  vecd<> info_vec = info_vector();
  
  spmatb robot_proj = seif::makeProjectionMatrix<N,J>(N+dim,
    std::vector<uint16_t>(), true);
  math::marginalizeInformationMatrixAndVector(info_mat, info_vec,
    robot_proj, epsilon);

  res.data.info_vector = std::vector<double>(info_vec.data()+N,
                                             info_vec.data()+dim+N);

  // fill info matrix data in response
  res.data.info_matrix.rows = dim;
  res.data.info_matrix.cols = dim;
  res.data.info_matrix.data.resize(info_mat.nonZeros());
  int i = 0;
  for(int k=0; k<info_mat.outerSize(); k++)
    for(spmatd::InnerIterator it(info_mat, k); it; ++it)
    {
      if(it.row() >= N && it.col() >= N)
      {
        res.data.info_matrix.data[i].row = it.row() - N;
        res.data.info_matrix.data[i].col = it.col() - N;
        res.data.info_matrix.data[i].value = it.value();
        i++;
      }
    }

  return res;
}

seif::Data
SEIFSlamDiffDrive::getMapDataFromExchangeMessage(
  MapExchangeSrv::Response const& message)
{
  int n = message.data.system_dim;
  seif::Data data;

  data.state = Eigen::Map<const vecd<>>(message.data.mean_vector.data(), n);
  data.info_vector = Eigen::Map<const vecd<>>(
    message.data.info_vector.data(), n);

  auto& triplets = message.data.info_matrix.data;
  std::vector<Eigen::Triplet<double>> triplets_list;
  int k = triplets.size();
  triplets_list.reserve(k);
  for(int i = 0; i < k; i++)
  {
    uint16_t row = triplets[i].row;
    uint16_t col = triplets[i].col;
    triplets_list.emplace_back(row, col, triplets[i].value);
  }

  data.info_matrix = spmatd(n, n);
  data.info_matrix.setFromTriplets(triplets_list.begin(), triplets_list.end());

  return data;
}

}
