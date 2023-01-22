#ifndef __SEIF_SLAM_HPP // Sparse Extended Information Filter
#define __SEIF_SLAM_HPP

#include "filter.hpp"
#include "seif_utils.hpp"
#include "landmarks_manager.hpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "nuslam/nuslam_system.hpp"
#include "rigid2d/rigid2d.hpp"
#include <math_utils/math.hpp>
#include <vector>
#include <tuple>
#include <stdexcept>

using math::vecd;
using math::matd;
using math::spmatd;
using math::spmatb;

namespace nuslam 
{
template <class ContainerAllocator>
struct MapExchangeSrvResponse_;
typedef MapExchangeSrvResponse_<std::allocator<void> > MapExchangeSrvResponse;

constexpr int DiffDriveControlDim = 2;
class SEIFSlamDiffDrive: public Filter<DiffDriveControlDim>
{
public:
  // robot state dim
  constexpr static int N = 3;
  // measurement dim
  constexpr static int J = 2;
  // control dim
  constexpr static int M = DiffDriveControlDim;

  constexpr static double epsilon = 1e-9;
  
  constexpr static double data_association_chi_threshold = 1.386; // Xi_{2, 0.50}
  
  const uint16_t map_size;

  SEIFSlamDiffDrive(NuslamSystem& system,
                   vecd<N> const& robot_init_state,
                   matd<N> const& robot_init_info,
                   matd<N> process_noise_covariance,
                   matd<J> measurement_noise_covariance,
                   uint16_t map_size,
                   uint16_t num_active_landmarks): 
    map_size{map_size}, 
    _landmarks_count{0},
    _system{system},
    _active_landmarks_manager{num_active_landmarks}
  {
    initInfoMatrix(robot_init_info);
    initStateAndInfoVector(robot_init_state);
    
    _process_noise_cov = process_noise_covariance;
    _process_noise_cov_inv = process_noise_covariance.inverse();
    _measurement_noise_cov = measurement_noise_covariance;
    _measurement_noise_cov_inv = measurement_noise_covariance.inverse();
    
    _initialized_landmark = std::vector<bool>(map_size, false);
    _landmarks_lut = std::vector<uint16_t>(map_size, 
      std::numeric_limits<uint16_t>::max());
  }
  
  static void transformInfoData(seif::Data&, math::MyTransformData const&);
  
  vecd<> const& 
  update(std::vector<std::tuple<uint16_t, vecd<2>>> const& landmarks) override;
  
  vecd<> const& predict(vecd<M> const&) override;

  Eigen::Ref<const vecd<>> state() const override
  {
    static vecd<> state;

    auto const& permanents =  _all_landmarks_manager.permanents();
    spmatd M = makeProjectionMatrix(permanents, true).cast<double>();
    state = M * _state; 
    
    return state; 
  }

  matd<> poseCovariance() const override
  {
    int n = N + J*_landmarks_count;
    matd<> info = _info_matrix.block(0,0,n,n);
    return info.inverse().block(0,0,N,N);
  }

  rigid2d::DiffDrive const& robot() const override
  {
    return _system.robot();
  }

  MapExchangeSrvResponse getExchangeMapData() const;
  static seif::Data getMapDataFromExchangeMessage(MapExchangeSrvResponse const&);

  void integrateMap(seif::Data const&);

  spmatd const& info_matrix() const
  {
    static spmatd info_matrix;
    
    auto const& permanents =  _all_landmarks_manager.permanents();
    spmatd M = makeProjectionMatrix(permanents, true).cast<double>();
    spmatd M_t = M.transpose();

    info_matrix = M * _info_matrix * M_t; 

    return info_matrix;
  }

  Eigen::Ref<const vecd<>> info_vector() const
  {
    static vecd<> info_vector;

    auto const& permanents =  _all_landmarks_manager.permanents();
    spmatd M = makeProjectionMatrix(permanents, true).cast<double>();
    info_vector = M * _info_vector; 
    
    return info_vector; 
  }

  int landmarks_count() const
  {
    return _all_landmarks_manager.permanents().size();
  }


  matd<> landmarkCovariance(int landmark_index) const override
  {
    // approximation
    int index = N + J*landmark_index;
    return _info_matrix.block(index, index, J, J).toDense().inverse();
  }

  friend seif::Data seif::mapInfoMerge(SEIFSlamDiffDrive const&, Data const&,
    std::vector<std::pair<uint16_t, uint16_t>> const&);

private:
  std::vector<bool> _initialized_landmark;
  std::vector<uint16_t> _landmarks_lut;
  unsigned int _landmarks_count;
  NuslamSystem _system;
  matd<N> _process_noise_cov;
  matd<N> _process_noise_cov_inv;
  matd<J> _measurement_noise_cov;
  matd<J> _measurement_noise_cov_inv;

  vecd<> _info_vector, _state;
  spmatd _info_matrix;

  seif::ActiveLandmarksBuffer _active_landmarks_manager;
  LandmarksManager _all_landmarks_manager;

  uint32_t internalStateSize() const
  {
    return N + J*_landmarks_count;
  }
  
  void initStateAndInfoVector(vecd<N> robot_init_state);
  void initInfoMatrix(matd<N> const&);

  seif::MotionUpdateVariables getMotionUpdateVariables(matd<N> const &);
  spmatd computePsi(matd<N> const &) const;
  spmatd computeLambda(spmatd const&) const;
  spmatd computeKappa(spmatd const&) const;
  void predictInfoVector(vecd<N> const&, seif::MotionUpdateVariables const &);
  
  void updateMeanVector(std::vector<uint16_t> const& landmark_index,
    bool update_robot_state=true,
    int max_iterations=5);

  uint16_t initLandmark(vecd<2> const&);
  void initLandmarkInInfoMat(std::pair<matd<J,N>, matd<J>> const&);
  void initlandmarkInInfoVector();

  void normalize_heading();

  void sparsify(std::vector<uint16_t> const&);
  
  static spmatd getSparseInfoMatrix(spmatd const&, std::vector<uint16_t> const&,
    std::vector<uint16_t> const&);

  static void sparsifyVector(vecd<>&, vecd<> const&, spmatd const&, spmatd const&);

  spmatb makeProjectionMatrix(std::vector<uint16_t> const&,
    bool robot=false) const;
  
  std::pair<std::vector<std::pair<uint16_t, uint16_t>>, std::vector<uint16_t> >
  dataAssociation(std::vector<vecd<J>> const&, 
    bool only_in_neighborhood=true) const;
  std::vector<uint16_t> getNeighborhoodMap() const;
  std::vector<matd<J>> getDataAssociationInvCovariances(
    std::vector<uint16_t> const&) const;
  std::vector<vecd<J>> getMeasurementPredictions(
    std::vector<uint16_t> const&) const;

  std::vector<uint16_t> _update(std::vector<vecd<2>> const& measurements, 
    std::vector<std::pair<uint16_t, uint16_t>> const& associations);
  
  void reduceInfoMatrixAndVectors(std::vector<uint16_t> const&);

  std::vector<uint16_t> getDataAssociationMarkovBlanket(
    uint16_t landmark_id,
    std::vector<uint16_t> const& active_landmarks,
    uint16_t& landmark_index_in_blanket) const;
};

} // end nuslam

#endif
