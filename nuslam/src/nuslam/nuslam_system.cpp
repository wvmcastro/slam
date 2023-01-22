#include "nuslam/nuslam_system.hpp"

#include <math_utils/math.hpp>
using math::matd;
using math::spmatd;

namespace nuslam
{
constexpr auto J = NuslamSystem::J;
constexpr auto N = NuslamSystem::N;

matd<N>
NuslamSystem::robotJacobian(vecd<> const& state, double previous_heading) const
{
  using std::cos; using std::sin; 
  auto wheels_dtheta = _drive.wheelsVelocities();
  double r = _drive.wheelRadius();
  double two_d = _drive.wheelBase();
  double alpha = (r / two_d) * (wheels_dtheta.right - wheels_dtheta.left);

  matd<N> G = matd<N>::Identity();
  if(rigid2d::almost_equal(alpha, 0, 1e-3)) // no angular velocity
  {
    double phi = state(0);
    double dx = wheels_dtheta.right * r;
    G(1, 0) = -dx*sin(phi); // dx / dphi
    G(2, 0) = dx*cos(phi); // dy / dphi
  }
  else
  {
    auto& u = wheels_dtheta;
    double f = (0.5 * two_d) * (u.right + u.left) / (u.right - u.left);
    G(1, 0) = f * ( cos(state(0)) - cos(previous_heading) ); // dx / dphi
    G(2, 0) = f * ( sin(state(0)) - sin(previous_heading) ); // dy / dphi
  }

  return G;
}

spmatd
NuslamSystem::lidarJacobian(Eigen::Ref<const vecd<>> state, int landmark_id,
  bool permanent_landmark)
{
  double phi = state(0);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);

  double d = scanner_displacement;
  // x coordinate of the base scanner link
  double xl = state(1) + d*cos_phi;
  // y coordinate of the base scanner link
  double yl = state(2) + d*sin_phi;

  int m = N + J*landmark_id; 
  double dx = state(m) - xl;
  double dy = state(m+1) - yl;
  double sq_range = dx*dx + dy*dy;
  
  int cols = state.size();
  spmatd H{J, cols};
  std::vector<int> reserve_size(cols, 0);
  reserve_size[0] = reserve_size[1] = reserve_size[2] = J;
  reserve_size[m] = reserve_size[m+1] = J;
  H.reserve(reserve_size);
  
  double inv_range = 1.0 / std::sqrt(sq_range);
  double inv_sq_range = 1.0 / sq_range;

  if(permanent_landmark == true)
  {
    // dRange/dPhi
    H.insert(0, 0) = (d * inv_range) * (dx*sin_phi - dy*cos_phi);
    // dRange/dx
    H.insert(0, 1) = -dx * inv_range;
    // dRange/dy
    H.insert(0, 2) = -dy * inv_range;
  
    // dBearing/dPhi
    H.insert(1, 0) = ( (-d * inv_sq_range)
        * (dy*sin_phi + dx*cos_phi) ) - 1.0;
    // dBearing/dx
    H.insert(1, 1) = dy * inv_sq_range;
    // dBearing/dy
    H.insert(1, 2) = -dx * inv_sq_range;
  }
  
  // dRange/dmj_x
  H.insert(0, m) = dx * inv_range;
  // dRange/dmj_y
  H.insert(0, m+1) = dy * inv_range;

  // dBearing/dmj_x
  H.insert(1, m) = -dy * inv_sq_range;
  // dBearing/dmj_y
  H.insert(1, m+1) = dx * inv_sq_range;

  return H;
}

vecd<J>
NuslamSystem::lidarMeasurementPrediction(vecd<> const& state, int landmark_id)
{
  // the measurement is made in the base scanner coordinate frame
  using std::cos; using std::sin;
  double heading = state(0);
  
  int m = N + 2*landmark_id;
  vecd<2> deltas;

  // x coordinate of the base scanner link
  double xl = state(1) + scanner_displacement*cos(heading);
  deltas(0) = state(m) - xl; // dx

  // y coordinate of the base scanner link
  double yl = state(2) + scanner_displacement*sin(heading);
  deltas(1) = state(m+1) - yl; // dy
  
  vecd<J> rtheta = math::cartesianToPolar(deltas);
  rtheta(1) = rigid2d::normalize_angle(rtheta(1) - heading); 
  return rtheta;
}

void
NuslamSystem::initLandmark(vecd<>& state, int landmark_id, 
    vecd<2> const& measurement)
{
    state.segment<J>(N + J*landmark_id) = landmarkEstimateFromMeasurement(
      state, measurement);
}

vecd<2>
NuslamSystem::landmarkEstimateFromMeasurement(vecd<> const & state, 
                                              vecd<J> const& measurement)
{
  auto& d = scanner_displacement;
  
  double xl = state(1) + d*std::cos(state(0));
  double yl = state(2) + d*std::sin(state(0));

  double alpha = rigid2d::normalize_angle(state(0) + measurement(1));

  vecd<2> landmark;
  landmark(0) = xl + measurement(0)*std::cos(alpha);
  landmark(1) = yl + measurement(0)*std::sin(alpha);

  return landmark;
}

std::pair<matd<J,N>, matd<J>>
NuslamSystem::invMeasurementModelJacobian(vecd<> const& state, 
                                          vecd<J> const& measurement,
                                          bool permanent_landmark)
{
  auto alpha = rigid2d::normalize_angle(state(0) + measurement(1));
  auto sin = std::sin(alpha);
  auto cos = std::cos(alpha);

  constexpr auto d = scanner_displacement;
  auto r = measurement(0);
  
  matd<J,N> Gr = matd<J,N>::Zero();
  if(permanent_landmark == true)
  {
    Gr(0,0) = -d*std::sin(state(0)) - r*sin;
    Gr(1,0) = d*std::cos(state(0)) + r*cos;
    Gr(0,1) = Gr(1,2) = 1.0;
  }

  matd<J,J> Gy{};
  Gy(0,0) = cos;
  Gy(0,1) = -r*sin;
  Gy(1,0) = sin;
  Gy(1,1) = r*cos;

  return std::make_pair(Gr, Gy);
}

} // end nuslam


