#include <catch_ros/catch.hpp>

#include <cstdlib>
#include <sstream>

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

TEST_CASE("Vector2D class")
{
  rigid2d::Vector2D v;
  rigid2d::Vector2D v1{1, 0};
  rigid2d::Vector2D v2{0, 1};

  SECTION("input and output operator")
  {
    auto vec_sb = "[1, 0]\0";
    SECTION("input with brackets")
    {
      std::stringstream in_s(vec_sb);
      in_s >> v;
      
      std::stringstream out_s;
      out_s << v;

      REQUIRE(out_s.str() ==  vec_sb);
    }

    SECTION("input withou brackets")
    {
      std::stringstream in_s("1 0");
      in_s >> v;

      std::stringstream out_s;
      out_s << v;

      REQUIRE(out_s.str() ==  vec_sb);
    }
  }

  SECTION("normalize method")
  {
    double x = ((double) std::rand()) / RAND_MAX;
    double y = ((double) std::rand()) / RAND_MAX;

    v = {x, y};
    v.normalize();
    auto norm = v.norm();

    REQUIRE(norm == Approx(1.0));
  }

  SECTION("+= operator")
  {
    v1 += v2;
    rigid2d::Vector2D sum{1, 1};
    REQUIRE(v1 == sum);
  }

  SECTION("+ operator")
  {
    auto vec = v1 + v2;
    rigid2d::Vector2D sum{1, 1};
    REQUIRE(vec == sum);
  }

  SECTION("-= operator")
  {
    v1 -= v2;
    rigid2d::Vector2D diff{1, -1};
    REQUIRE(diff == v1);
  }

  SECTION("- operator")
  {
    auto vec = v1 - v2;
    rigid2d::Vector2D diff{1, -1};
    REQUIRE(diff == vec);
  }

  SECTION("*= operator")
  {
    rigid2d::Vector2D vec{2, 0};
    v1 *= 2;
    REQUIRE(v1 == vec);
  }
   
  SECTION("* operator")
  {
    auto vec = 2 * v1;
    REQUIRE((2*v1) == vec);
  }

  SECTION ("angle method")
  {
    rigid2d::Vector2D vpi{-1, 0};
    REQUIRE(vpi.angle() == Approx(math::PI));
    REQUIRE(v1.angle() == Approx(0));
  }
}

TEST_CASE("Transform2D class")
{
  rigid2d::Transform2D tr;
  tr.setTR(-1, 3, rigid2d::deg2rad(90));

  SECTION("input and output functions")
  {
    std::stringstream in1_s("45.0 0.5 0.8");
    std::stringstream in2_s("-45.0\n0.5\n-0.8");
    std::stringstream in3_s("30 -0.5 0.8");
    
    std::stringstream out_s;
    in1_s >> tr;
    out_s << tr;
    REQUIRE(out_s.str() == "(degrees): 45 dx: 0.5 dy: 0.8");

    in2_s >> tr;
    out_s.str("");
    out_s << tr;
    REQUIRE(out_s.str() == "(degrees): -45 dx: 0.5 dy: -0.8");
    
    in3_s >> tr;
    out_s.str("");
    out_s << tr;
    REQUIRE(out_s.str() == "(degrees): 30 dx: -0.5 dy: 0.8");
  }

  SECTION("transform vector")
  {
   
    rigid2d::Vector2D vc{3, 3};
    auto va = tr(vc);

    std::stringstream va_s;
    va_s << va;

    REQUIRE(va_s.str() == "[-4, 6]");

    vc.x = 0;
    vc.y = 0;
    tr.setTR(3, 3, rigid2d::deg2rad(45));
    auto va2 = tr(vc);
    va_s.str("");
    va_s << va2;
    REQUIRE(va_s.str() == "[3, 3]");
  }

  SECTION("inverse transform")
  {
    auto Tca = tr.inv();
    std::stringstream tca_s;
    tca_s << Tca;

    REQUIRE(tca_s.str() == "(degrees): -90 dx: -3 dy: -1");
  }

  // This test is from: 
  // https://github.com/moribots/turtlebot3_from_scratch/blob/master/rigid2d/tests/rigid2d_test.cpp
  SECTION("integrate twist")
  {
    // w_z = 0
    rigid2d::Transform2D Tac;
    std::string input1 = "90 -1 3";
    std::stringstream in_s1(input1);
    in_s1 >> Tac;
    
    SECTION("pure translation")
    {
      rigid2d::Twist tw(0, 1, 1);

      // Integrate
      auto Tac_twisted = Tac.integrateTwistForOneTimeUnit(tw);

      std::string output1 = "(degrees): 90 dx: -2 dy: 4";
      std::stringstream out_s1;
      
      out_s1 << Tac_twisted;

      // compare raw output to out_s input
      REQUIRE(out_s1.str() == output1);
    }
    
    SECTION("non-zero angular velocity")
    {
      SECTION("zero vy")
      {
        rigid2d::Twist tw2(math::PI/3.0, math::PI/3.0, 0);

        // Integrate
        auto Tac_twisted = Tac.integrateTwistForOneTimeUnit(tw2);

        std::string output2 = "(degrees): 150"
                              " dx: -1.5 dy: 3.86603";
        std::stringstream out_s2;
        out_s2 << Tac_twisted;
      }

      SECTION("non zero vy")
      {
        rigid2d::Twist tw2(1, 1, 1);

        // Integrate
        auto Tac_twisted = Tac.integrateTwistForOneTimeUnit(tw2);

        std::string output2 = "(degrees): 147.296"
                              " dx: -2.30117 dy: 3.38177";
        std::stringstream out_s2;
        out_s2 << Tac_twisted;

        // compare raw output to out_s input
        REQUIRE(out_s2.str() ==  output2);
      }
    }

    SECTION("trivial zero twist")
    {
      // Zero Twist
      rigid2d::Twist tw3(0, 0, 0);

      // Integrate
      auto Tac_twisted = Tac.integrateTwistForOneTimeUnit(tw3);

      std::string output3 = "(degrees): 90 dx: -1 dy: 3";
      std::stringstream out_s3;
      out_s3 << Tac_twisted;

      // compare raw output to out_s input
      REQUIRE(out_s3.str() == output3);
    }
  }
}

TEST_CASE("normalize angle function")
{
  SECTION("test cases")
  {
    using namespace rigid2d;
    double theta = normalize_angle(deg2rad(270));
    REQUIRE(theta == Approx(-0.5*math::PI));

    theta = normalize_angle(deg2rad(-270));
    REQUIRE(theta == Approx(0.5*math::PI));

    theta = normalize_angle(deg2rad(190));
    REQUIRE(theta == Approx(deg2rad(-170)));

    theta = normalize_angle(deg2rad(18035)); // 35 deg
    REQUIRE(theta == Approx(deg2rad(35)));

    theta = normalize_angle(deg2rad(-18035)); // -35deg
    REQUIRE(theta == Approx(deg2rad(-35)));

    theta = normalize_angle(deg2rad(17965)); // -35deg
    REQUIRE(theta == Approx(deg2rad(-35)));

  }
}

TEST_CASE("diff drive")
{
  struct rigid2d::Pose p;
  p = {0, 0, 0};
  auto drive = rigid2d::DiffDrive(p, 0.08, 0.02);

  SECTION("no input")
  {
    auto previous_pose = drive.pose();
    auto delta_pose = drive.computeMotionDelta(0, 0);

    auto pose = drive.updateOdometry(0, 0);
    REQUIRE(pose.theta == Approx(0.0));
    REQUIRE(pose.x == Approx(0));
    REQUIRE(pose.y == Approx(0));
    
    REQUIRE((previous_pose.theta + delta_pose[0]) == Approx(0.0));
    REQUIRE((previous_pose.x + delta_pose[1]) == Approx(0));
    REQUIRE((previous_pose.y + delta_pose[2]) == Approx(0));
  }

  SECTION("pure translation")
  {
    auto previous_pose = drive.pose();
    auto delta_pose = drive.computeMotionDelta(2, 2);

    drive.updateOdometry(2, 2);
    auto pose = drive.pose();
    REQUIRE(pose.theta == 0);
    REQUIRE(pose.x == 0.04);
    REQUIRE(pose.y == 0);

    REQUIRE((previous_pose.theta + delta_pose[0]) == 0);
    REQUIRE((previous_pose.x + delta_pose[1]) == 0.04);
    REQUIRE((previous_pose.y + delta_pose[2]) == 0);
  }

  SECTION("pure rotation")
  {
    auto previous_pose = drive.pose();
    auto delta_pose = drive.computeMotionDelta(-math::PI, math::PI);

    drive.updateOdometry(-math::PI, math::PI);
    auto pose = drive.pose();
    REQUIRE(pose.theta == 0.5*math::PI);
    REQUIRE(pose.x == 0);
    REQUIRE(pose.y == 0);

    REQUIRE((previous_pose.theta + delta_pose[0]) == 0.5*math::PI);
    REQUIRE((previous_pose.x + delta_pose[1]) == 0);
    REQUIRE((previous_pose.y + delta_pose[2]) == 0);
  }

  SECTION("mixed rotation and translation")
  {
    p.x = 0.14;
    p.theta = 0.5*math::PI;
    drive = rigid2d::DiffDrive(p, 0.08, 0.02); 

    auto previous_pose = drive.pose();
    auto delta_pose = drive.computeMotionDelta((5.0/12.0) * math::PI,
                                               (9.0/12.0) * math::PI);

    auto pose = drive.updateOdometry((5.0/12.0) * math::PI, 
                                     (9.0/12.0) * math::PI);
    CHECK(pose.theta == Approx(math::PI* (7.0/12.0)));
    CHECK(pose.x == Approx(0.135229616));
    CHECK(pose.y == Approx(0.036234666));

    CHECK((previous_pose.theta + delta_pose[0]) == Approx(math::PI* (7.0/12.0)));
    CHECK((previous_pose.x + delta_pose[1]) == Approx(0.135229616));
    CHECK((previous_pose.y + delta_pose[2]) == Approx(0.036234666));
  }

}

