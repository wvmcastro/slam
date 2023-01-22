#ifndef NUSLAM_SYSTEM_HPP
#define NUSLAM_SYSTEM_HPP 

#include "rigid2d/diff_drive.hpp"
#include "nuslam/filter.hpp"
#include <math_utils/math.hpp>


namespace nuslam
{

class NuslamSystem
{
public:
  constexpr static int N = 3;
  constexpr static int M = 2;
  constexpr static int J = 2; // measurement dim
  constexpr static double scanner_displacement = -0.032;

  NuslamSystem(vecd<N> robot_init_pose,
               float wheel_base, 
               float wheel_radius):
    _wheel_base{wheel_base}, _wheel_radius{wheel_radius}
  {
    auto& x0 = robot_init_pose;
    rigid2d::Pose p = {x0(1), x0(2), x0(0)};
    _drive = rigid2d::DiffDrive(p, wheel_base, wheel_radius);
  }

  math::matd<N> robotJacobian(vecd<> const&, double previous_heading) const;
  static math::spmatd lidarJacobian(Eigen::Ref<const vecd<>>, int, bool=true);
  static math::vecd<J> lidarMeasurementPrediction(vecd<> const&, int);
  void initLandmark(vecd<>&, int, vecd<2> const&);
  
  static vecd<2> landmarkEstimateFromMeasurement(vecd<> const& state, 
                                                 vecd<2> const& measurement);
  
  std::pair<matd<J,N>, matd<J>>
  static invMeasurementModelJacobian(vecd<> const&, vecd<J> const&, bool=true);

  rigid2d::DiffDrive& robot()
  {
    return _drive;
  }

  rigid2d::DiffDrive const& robot() const
  {
    return _drive;
  }

  uint16_t landmarkIdToVectorStateIndex(uint16_t id)
  {
    return N + id*J;
  }
private:
  double _wheel_base, _wheel_radius;
  rigid2d::DiffDrive _drive;
};

} // end nuslam

#endif
