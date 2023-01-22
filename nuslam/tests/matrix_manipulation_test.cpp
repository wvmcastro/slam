#include <catch_ros/catch.hpp>
#include <math_utils/math.hpp>

TEST_CASE("row and columns deletions")
{
  math::mat<int> m(3,3);
  m << 1, 2, 3, 
       4, 5, 6, 
       7, 8, 9;
  
  SECTION("delete row")
  {
    math::delete_row(m, 0);
    
    math::mat<int, 2, 3> result;
    result << 4, 5, 6,
              7, 8, 9;

    REQUIRE(m == result);
  }
  
  SECTION("delete column")
  {
    math::delete_col(m, 1);
    
    math::mat<int, 3, 2> result;
    result << 1, 3,
              4, 6,
              7, 9;

    REQUIRE(m == result);
  }
  
  SECTION("delete row and column")
  {
    math::delete_col(m, 1);
    math::delete_row(m, 1);
    
    math::mat<int, 2, 2> result;
    result << 1, 3,
              7, 9;

    REQUIRE(m == result);
    REQUIRE(m.rows() == 2);
    REQUIRE(m.cols() == 2);
  }
}

