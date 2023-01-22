#include <catch_ros/catch.hpp>
#include <graphics_utils/bresenham.hpp>
#include <math_utils/math.hpp>
#include <vector>

using math::veci;

TEST_CASE("positive small slope")
{
  SECTION("zero slope")
  {
    veci<2> start{2,2};
    veci<2> end{20, 2};

    auto cells = bresenham(start, end);

    std::vector<veci<2>> expected;
    for(int i = 2; i < end[0]+1; i++)
      expected.emplace_back(i, 2);

    int n = expected.size(); 
    for(int i = 0; i < n; i++)
    {
      REQUIRE(cells[i] == expected[i]);
    }
  }

  SECTION("0 < slope < 1")
  {
    std::vector<veci<2>> expected = {{0, 5}, {1, 6}, {2, 7}, {3, 8}, {4, 9}, 
                                     {5, 10}, {6, 11}, {7, 12}, {8, 12}, 
                                     {9, 13}, {10, 14}, {11, 15}, {12, 16}, 
                                     {13, 17}, {14, 18}, {15, 19}};
    auto cells = bresenham(expected[0], expected[expected.size()-1]);

    int n = expected.size();
    for(int i = 0; i < n; i++)
      REQUIRE(cells[i] == expected[i]);
  }

  SECTION("slope = 1")
  {
    veci<2> start{2,2};
    veci<2> end{20,20};

    auto cells = bresenham(start, end);

    std::vector<veci<2>> expected;
    for(int i = start[0]; i < end[0]+1; i++)
      expected.emplace_back(i, i);
    
    int n = expected.size();
    for(int i = 0; i < n; i++)
      REQUIRE(cells[i] == expected[i]);
  }
}

TEST_CASE("positive big slope")
{

  SECTION("infinite slope")
  {
    veci<2> start{2,2};
    veci<2> end{2, 20};

    auto cells = bresenham(start, end);

    std::vector<veci<2>> expected;
    for(int i = 2; i < end[1]+1; i++)
      expected.emplace_back(2,i);
    
    int n = expected.size(); 
    for(int i = 0; i < n; i++)
    {
      REQUIRE(cells[i] == expected[i]);
    }
  }

  SECTION("1 < slope < inf")
  {
    veci<2> start{14,8};
    veci<2> end{15,16};

    std::vector<veci<2>> expected;
    expected.push_back(start);
    expected.emplace_back(14, 9); 
    expected.emplace_back(14, 10);
    expected.emplace_back(14, 11);
    expected.emplace_back(15, 12);
    expected.emplace_back(15, 13);
    expected.emplace_back(15, 14); 
    expected.emplace_back(15, 15);
    expected.push_back(end);

    auto cells = bresenham(start, end);
    
    int n = expected.size(); 
    for(int i = 0; i < n; i++)
    {
      REQUIRE(cells[i] == expected[i]);
    }
  }
}

TEST_CASE("negative small slope")
{
  SECTION("-1 < slope < 0")
  {
    // [(1, 2), (2, 1), (3, 1), (4, 0)]
    std::vector<veci<2>> expected = {{1, 2}, {2, 1}, {3, 1}, {4, 0}};
    auto cells = bresenham(expected[0], expected[expected.size()-1]);

    int n = expected.size();
    for(int i = 0; i < n; i++)
      REQUIRE(cells[i] == expected[i]);
  }
}

TEST_CASE("negative big slope")
{
  SECTION("-inf < slope < -1")
  {
    std::vector<veci<2>> expected = {{5, 0}, {4, 1}, {4, 2}, 
                                     {3, 3}, {3, 4}, {2, 5}, {2, 6}};
    auto cells = bresenham(expected[0], expected[expected.size()-1]);

    int n = expected.size();
    for(int i = 0; i < n; i++)
      REQUIRE(cells[i] == expected[i]);
  }
  
}