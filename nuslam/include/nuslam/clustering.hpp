#ifndef __CLUSTERING_HPP
#define __CLUSTERING_HPP
#include <ros/ros.h>
#include <vector>
#include <math_utils/math.hpp>
using namespace math;

namespace clustering
{
template <typename T = float, int q>
mat<T>
distance_matrix(std::vector<vec<T, q>> const& s1, 
                std::vector<vec<T, q>> const& s2,
                bool border_index = false)
{
  mat<T> d;
  int m, n;
  if(border_index == true) 
  {
    m = s1.size() + 1;
    n = s2.size() + 1;
    d = mat<T>::Zero(m, n);
    d(0, 0) = -1;  
    for(int i = 1; i < n; i++) d(0, i) = i - 1;
    for(int i = 1; i < m; i++) d(i, 0) = i - 1;
  }
  else 
  {
    m = s1.size();
    n = s2.size();
    d = mat<T>::Zero(m, n);
  }
  
  int offset = (int) border_index;
  for(int j = offset; j < n; j++)
    for(int i = offset; i < m; i++)
      d(i, j) = distance_l2<q>(s1[i-offset], s2[j-offset]);

  return d;
}

template <typename T1, typename T2, int m>
std::vector<vec<int, 2>>
cluster_in_pairs(std::vector<vec<T1, m>> const& s1, 
                 std::vector<vec<T2, m>> const& s2,
                 double max_distance_threshold,
                 std::vector<int>& not_clustered_s1, 
                 std::vector<int>& not_clustered_s2)
{
  auto d = distance_matrix(s1, s2, true);  
  std::vector<vec<int, 2>> pairs;
  
  auto s1_size = s1.size();
  auto s2_size = s2.size();
  int n = std::min(s1_size, s2_size);
  while(n)
  {
    Eigen::Index min_row, min_col;
    int rows = d.rows() - 1;
    int cols = d.cols() - 1;
    auto distance = d.block(1, 1, rows, cols).minCoeff(&min_row, &min_col);
    
    if(distance > max_distance_threshold)
      break;
    
    pairs.push_back(vec<int, 2>(d(min_row+1, 0), d(0, min_col+1)));

    delete_row(d, min_row+1);
    delete_col(d, min_col+1);
    n--;
  }
  
  for(int i = 1; i < d.cols(); i++)
    not_clustered_s2.push_back(d(0, i));

  for(int i = 1; i < d.rows(); i++)
    not_clustered_s1.push_back(d(i, 0));

  return pairs;
}

}
#endif
