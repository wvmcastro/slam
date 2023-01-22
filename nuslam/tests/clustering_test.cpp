#include <catch_ros/catch.hpp>
#include <nuslam/clustering.hpp>

TEST_CASE("distance l2 for one dim points")
{
  REQUIRE(math::distance_l2<1>(vec<float, 1>(1), vec<float, 1>(8)) == 7);
}


TEST_CASE("distance matrix for one dim points")
{
  SECTION("s1 < s2 with border")
  {
    std::vector<vec<float, 1>> s1;
    s1.push_back(vec<float, 1>(1));
    s1.push_back(vec<float, 1>(5));

    float s2_points[] = {2, 4, 7, 8};
    std::vector<vec<float, 1>> s2(s2_points, s2_points+4);

    auto d = clustering::distance_matrix(s1, s2, true);

    mat<float> my_d(s1.size()+1, s2.size() + 1);
    my_d << -1, 0, 1, 2, 3,
             0, 1, 3, 6, 7,
             1, 3, 1, 2, 3;

    REQUIRE(d == my_d);
  }
  
  SECTION("s1 < s2 without border")
  {
    std::vector<vec<float, 1>> s1;
    s1.push_back(vec<float, 1>(1));
    s1.push_back(vec<float, 1>(5));

    float s2_points[] = {2, 4, 7, 8};
    std::vector<vec<float, 1>> s2(s2_points, s2_points+4);

    auto d = clustering::distance_matrix(s1, s2, false);

    mat<float> my_d(s1.size(), s2.size());
    my_d << 1, 3, 6, 7,
            3, 1, 2, 3;

    REQUIRE(d == my_d);
  }
}

TEST_CASE("one dimensional clustering")
{
  SECTION("s1 < s2")
  {
    std::vector<vec<float, 1>> s1;
    s1.push_back(vec<float, 1>(1));
    s1.push_back(vec<float, 1>(5));

    float s2_points[] = {2, 4, 7, 8};
    std::vector<vec<float, 1>> s2(s2_points, s2_points+4);
    
    std::vector<int> s1_not_clustered;
    std::vector<int> s2_not_clustered;

    auto pairs = clustering::cluster_in_pairs(s1, s2, 
        1.1, s1_not_clustered, s2_not_clustered);

    REQUIRE(s1_not_clustered.size() == 0);
    REQUIRE(s2_not_clustered.size() == 2);

    std::vector<int> index(2);
    index[0] = 2;
    index[1] = 3;

    REQUIRE(s2_not_clustered == index);

    std::vector<vec<int, 2>> sol_pairs;
    sol_pairs.push_back(vec<int, 2>(0,0));
    sol_pairs.push_back(vec<int, 2>(1,1));
    REQUIRE(sol_pairs == pairs);
  }
  
  SECTION("one of the vectors is empty")
  {
    std::vector<vec<float, 1>> s1;

    float s2_points[] = {2, 4, 7, 8};
    std::vector<vec<float, 1>> s2(s2_points, s2_points+4);
    
    std::vector<int> s1_not_clustered;
    std::vector<int> s2_not_clustered;

    auto pairs = clustering::cluster_in_pairs(s1, s2, 
        1.1, s1_not_clustered, s2_not_clustered);

    REQUIRE(pairs.size() == 0);
    REQUIRE(s1_not_clustered.size() == 0);
    REQUIRE(s2_not_clustered.size() == 4);

    std::vector<int> index(4);
    index[0] = 0;
    index[1] = 1;
    index[2] = 2;
    index[3] = 3;

    REQUIRE(s2_not_clustered == index);
  }
}

TEST_CASE("two dimensional clustering")
{
  SECTION("s1 < s2")
  {
    std::vector<vec<float, 2>> s1;
    s1.push_back(vec<float, 2>(1, 1));
    s1.push_back(vec<float, 2>(1, 5));

    float s2_points[] = {2, 4, 7, 8};
    std::vector<vec<float, 2>> s2;
    for(auto p : s2_points)
      s2.push_back(vec<float, 2>(1, p));
    
    std::vector<int> s1_not_clustered;
    std::vector<int> s2_not_clustered;

    auto pairs = clustering::cluster_in_pairs(s1, s2, 
        1.1, s1_not_clustered, s2_not_clustered);

    REQUIRE(s1_not_clustered.size() == 0);
    REQUIRE(s2_not_clustered.size() == 2);

    std::vector<int> index(2);
    index[0] = 2;
    index[1] = 3;

    REQUIRE(s2_not_clustered == index);

    std::vector<vec<int, 2>> sol_pairs;
    sol_pairs.push_back(vec<int, 2>(0,0));
    sol_pairs.push_back(vec<int, 2>(1,1));
    REQUIRE(sol_pairs == pairs);
  }
}
