#include <catch_ros/catch.hpp>
#include <nuslam/geometry.h>
#include <eigen3/Eigen/Dense>

TEST_CASE("circle fit function")
{
  SECTION("teste #1")
  {
    Eigen::ArrayXf x(6); 
    x << 1, 2, 5, 7, 9, 3;

    Eigen::ArrayXf y(6);
    y << 7, 6, 8, 7, 5, 7;

    auto c = circle_fit(x, y);
    float cx = c(0);
    float cy = c(1);
    float r = c(2);
    REQUIRE(cx == Approx(4.615482).epsilon(1e-4));
    REQUIRE(cy == Approx(2.807354).epsilon(1e-4));
    REQUIRE(r == Approx(4.8275).epsilon(1e-4));
  }
  SECTION("teste #2")
  {
    Eigen::ArrayXf x(4); 
    x << -1, -0.3, 0.3, 1;

    Eigen::ArrayXf y(4);
    y << 0, -0.06, 0.1, 0;

    auto c = circle_fit(x, y);
    float cx = c(0);
    float cy = c(1);
    float r = c(2);
    REQUIRE(cx == Approx(0.4908357).epsilon(1e-4));
    REQUIRE(cy == Approx(-22.15212).epsilon(1e-4));
    REQUIRE(r == Approx(22.17979).epsilon(1e-4));
  }
}

