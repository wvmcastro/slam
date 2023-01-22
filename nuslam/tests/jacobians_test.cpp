#include <catch_ros/catch.hpp>
#include "nuslam/nuslam_system.hpp"


TEST_CASE("inverse lidar model jacobian")
{
  using namespace nuslam;
  vecd<3> state = vecd<3>::Random();
  vecd<2> measurement = vecd<2>::Random();

  auto jacobians = NuslamSystem::invMeasurementModelJacobian(state, 
    measurement);
  double eps = 1e-7;
  
  SECTION("robot jacobian")
  {
    auto pred = NuslamSystem::landmarkEstimateFromMeasurement(state, 
      measurement);

    matd<2,3> Gr_numeric{};
    
    vecd<3> state_phi{state(0) + eps, state(1), state(2)};
    Gr_numeric.block<2,1>(0,0) = NuslamSystem::landmarkEstimateFromMeasurement(
      state_phi, measurement) - pred;

    vecd<3> state_x{state(0), state(1) + eps, state(2)};
    Gr_numeric.block<2,1>(0, 1) = NuslamSystem::landmarkEstimateFromMeasurement(
      state_x, measurement) - pred;

    vecd<3> state_y{state(0), state(1), state(2) + eps};
    Gr_numeric.block<2,1>(0,2) = NuslamSystem::landmarkEstimateFromMeasurement(
      state_y, measurement) - pred;
    
    Gr_numeric /= eps;

    auto& Gr_analytic = jacobians.first;

    REQUIRE(math::allClose(Gr_analytic, Gr_numeric) == true);
    REQUIRE(math::allClose(Gr_numeric, Gr_analytic) == true);
  }

  SECTION("measurement jacobian")
  {
    auto pred_landmark = NuslamSystem::landmarkEstimateFromMeasurement(
      state, measurement);

    matd<2,2> Gy_numeric{};
    vecd<2> measurement_theta = vecd<2>{measurement(0) + eps, measurement(1)};

    Gy_numeric.block<2,1>(0, 0) = NuslamSystem::landmarkEstimateFromMeasurement(
      state, measurement_theta) - pred_landmark;

    vecd<2> measurement_r = vecd<2>{measurement(0), measurement(1) + eps};
    Gy_numeric.block<2,1>(0,1) = NuslamSystem::landmarkEstimateFromMeasurement(
      state, measurement_r) - pred_landmark;
    
    Gy_numeric /= eps;

    auto& Gy_analytic = jacobians.second;

    REQUIRE(math::allClose(Gy_analytic, Gy_numeric) == true);
    REQUIRE(math::allClose(Gy_numeric, Gy_analytic) == true);
  }
}


TEST_CASE("robot model jacobian")
{
  using namespace nuslam;

  vecd<3> x0 = vecd<3>::Random();
  float wheel_base = 0.16;
  float wheel_radius = 0.033;
  NuslamSystem system{x0, wheel_base, wheel_radius};
  
  double eps = 1e-7;
  
  SECTION("with angular velocity")
  {
    vecd<2> wheels_position = vecd<2>::Random();
    wheels_position(0) += 1e-3;

    float old_phi = x0(0);
    auto pose = system.robot().updateOdometry(wheels_position(0),
      wheels_position(1));
    vecd<3> state{};
    state(0) = rigid2d::normalize_angle(pose.theta);
    state(1) = pose.x;
    state(2) = pose.y;
    auto Fx_analytic = system.robotJacobian(state, old_phi);

    matd<3,3> Fx_numeric{};
    rigid2d::Pose pose_phi = {x0(1), x0(2), x0(0)+eps};
    rigid2d::DiffDrive robot{pose_phi, wheel_base, wheel_radius};
    auto pose_phi_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dphi{pose_phi_pred.theta, pose_phi_pred.x, pose_phi_pred.y};
    Fx_numeric.block<3,1>(0,0) = dphi - state;
    
    rigid2d::Pose pose_x = {x0(1)+eps, x0(2), x0(0)};
    robot = rigid2d::DiffDrive(pose_x, wheel_base, wheel_radius);
    auto pose_x_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dx{pose_x_pred.theta, pose_x_pred.x, pose_x_pred.y};
    Fx_numeric.block<3,1>(0,1) = dx - state;
    
    rigid2d::Pose pose_y = {x0(1), x0(2)+eps, x0(0)};
    robot = rigid2d::DiffDrive(pose_y, wheel_base, wheel_radius);
    auto pose_y_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dy{pose_y_pred.theta, pose_y_pred.x, pose_y_pred.y};
    Fx_numeric.block<3,1>(0,2) = dy - state;

    Fx_numeric /= eps;
    
    REQUIRE(math::allClose(Fx_numeric, Fx_analytic));
    REQUIRE(math::allClose(Fx_analytic, Fx_numeric));

  }

  SECTION("no angular velocity")
  {
    vecd<2> wheels_position = vecd<2>::Random();
    wheels_position(0) = wheels_position(1);

    float old_phi = x0(0);
    auto pose = system.robot().updateOdometry(wheels_position(0),
      wheels_position(1));
    vecd<3> state{};
    state(0) = rigid2d::normalize_angle(pose.theta);
    state(1) = pose.x;
    state(2) = pose.y;
    auto Fx_analytic = system.robotJacobian(state, old_phi);

    matd<3,3> Fx_numeric{};
    rigid2d::Pose pose_phi = {x0(1), x0(2), x0(0)+eps};
    rigid2d::DiffDrive robot{pose_phi, wheel_base, wheel_radius};
    auto pose_phi_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dphi{pose_phi_pred.theta, pose_phi_pred.x, pose_phi_pred.y};
    Fx_numeric.block<3,1>(0,0) = dphi - state;
    
    rigid2d::Pose pose_x = {x0(1)+eps, x0(2), x0(0)};
    robot = rigid2d::DiffDrive(pose_x, wheel_base, wheel_radius);
    auto pose_x_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dx{pose_x_pred.theta, pose_x_pred.x, pose_x_pred.y};
    Fx_numeric.block<3,1>(0,1) = dx - state;
    
    rigid2d::Pose pose_y = {x0(1), x0(2)+eps, x0(0)};
    robot = rigid2d::DiffDrive(pose_y, wheel_base, wheel_radius);
    auto pose_y_pred = robot.updateOdometry(wheels_position(0), wheels_position(1));
    vecd<3> dy{pose_y_pred.theta, pose_y_pred.x, pose_y_pred.y};
    Fx_numeric.block<3,1>(0,2) = dy - state;

    Fx_numeric /= eps;
    
    REQUIRE(math::allClose(Fx_numeric, Fx_analytic));
    REQUIRE(math::allClose(Fx_analytic, Fx_numeric));
  }
}

TEST_CASE("lidar measurement model jacobian")
{
  using namespace nuslam;
  constexpr int N = 3;
  constexpr int map_size = 5;
  constexpr int n = N + 2*map_size;
  vecd<n> state = vecd<n>::Random();
  state(0) = rigid2d::normalize_angle(state(0));
  
  int landmark_index = 2;
  auto pred0 = NuslamSystem::lidarMeasurementPrediction(state, landmark_index);
  
  double eps = 1e-7;
  matd<2,n> Hx_numeric{}; 
  for(int i = 0; i < n; i++)
  {
    vecd<n> dstate = state;
    dstate(i) += eps;
    if(i == 0) dstate(0) = rigid2d::normalize_angle(dstate(0));
    auto dpred = NuslamSystem::lidarMeasurementPrediction(dstate, landmark_index);
    Hx_numeric.block<2,1>(0,i) = dpred - pred0;
  }

  Hx_numeric /= eps;

  matd<2,n> Hx_analytic = NuslamSystem::lidarJacobian(state, landmark_index);
  REQUIRE(math::allClose(Hx_numeric, Hx_analytic));
  REQUIRE(math::allClose(Hx_analytic, Hx_numeric));
}