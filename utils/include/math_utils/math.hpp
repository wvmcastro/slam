#ifndef __MATH_HPP_
#define __MATH_HPP_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <misc_utils/misc.hpp>
#include <ros/ros.h>
#include <utils/Matrix2D.h>

template <typename T>
std::basic_ostream<char>& operator<<(std::basic_ostream<char> &ss, 
  std::vector<T> const &v)
{
  int n = v.size();
  if(n == 0)
    return ss << "[]";

  n -= 1;
  ss << " [";
  for(int i = 0; i < n; i++)
    ss << v[i] << ", ";
  ss << v[n] << "]";
  return ss;
}

template <typename T>
std::basic_ostream<char>& operator<<(std::basic_ostream<char> &ss, 
  Eigen::Triplet<T> const &t)
{
  ss << "{ " << "(" << t.row() << "," << t.col() << "), " << t.value() << " }";
  return ss;
}

namespace math
{

  namespace constants
  {
    constexpr double pi = 3.14159265358979323846;
  }

template <typename T, int n = Eigen::Dynamic>
using vec = Eigen::Matrix<T, n, 1>;

template <int n = Eigen::Dynamic>
using vecf = vec<float, n>;

template <int n = Eigen::Dynamic>
using vecd = vec<double, n>;

template <int n = Eigen::Dynamic>
using veci = vec<int, n>;

template <typename T, int m = Eigen::Dynamic, int n = Eigen::Dynamic>
using mat = Eigen::Matrix<T, m, n, Eigen::ColMajor>;

template <int m = Eigen::Dynamic, int n = m>
using matf = Eigen::Matrix<float, m, n, Eigen::ColMajor>;

template <int m = Eigen::Dynamic, int n = m>
using matd = Eigen::Matrix<double, m, n, Eigen::ColMajor>;

using mat3d = matd<3>;

template <typename T>
using spmat = Eigen::SparseMatrix<T, Eigen::ColMajor>;

using spmatd = spmat<double>;
using spmati = spmat<int>;
using spmatb = spmat<bool>;

template <typename T>
struct Point
{
  T x;
  T y;

  Point()
  {
    // do nothing
  }

  Point(T x_, T y_) : x{x_}, y{y_}
  {
    // do nothing
  }
};

struct MyTransformData
{
  MyTransformData()
  {
    // do nothing
  }

  MyTransformData(double x, double y, double theta)
    : translation{x, y}, rotation{theta}
  {
    // do nothing
  }

  struct Point<double> translation;
  double rotation;
};

template <typename T>
mat<T, 2, 2>
rotationMatrix2D(T yaw)
{
  mat<T, 2, 2> rot;
  rot(0,0) = rot(1,1) = cos(yaw);
  rot(0,1) =  sin(yaw);
  rot(1,0) = -rot(0,1);
  return rot;
}

template <typename T>
using point = Point<T>;

using pointf = Point<float>;

template <int n, typename T1, typename T2>
double 
distance_l2(T1 const& v1, T2 const& v2)
{
  double q = 0;
  for(int i = 0; i < n; i++)
    q += std::pow(v1[i] - v2[i], 2);
  
  return std::sqrt(q);
}

template <int n>
uint32_t
distance_l1(veci<n> const& v1, veci<n> const& v2)
{
  return (v1 - v2).cwiseAbs().sum();
}


template <typename T>
void
delete_row(mat<T>& M, int row_index)
{
  int m = M.rows();
  int n = M.cols();

  M.block(row_index, 0, m-row_index-1, n) = 
    M.block(row_index+1, 0, m-row_index-1, n);

  M.conservativeResize(m-1, n);
}


template <typename T>
void
delete_col(mat<T>& M, int col_index)
{
  int m = M.rows();
  int n = M.cols();
  
  M.block(0, col_index, m, n-col_index-1) = 
    M.block(0, col_index+1, m, n-col_index-1);

  M.conservativeResize(m, n-1);
}

inline vecd<2>
cartesianToPolar(vecd<2> const& cartesian_landmark)
{
  double x = cartesian_landmark(0);
  double y = cartesian_landmark(1);

  double r = std::sqrt(x*x + y*y);
  double theta = std::atan2(y, x);

  return vecd<2>(r, theta);
}

template <typename T>
inline double
cond(mat<T> const& m)
{
  Eigen::JacobiSVD<mat<T>> svd(m);
  double cond = svd.singularValues()(0) 
    / svd.singularValues()(svd.singularValues().size() - 1);

  return cond;
}

template <typename T>
void 
polarToCartesian(std::vector<T>& points)
{
  constexpr int r = 0;
  constexpr int theta = 1;
  for(auto& point : points)
  {
    double range = point[r];
    point[r] = range * std::cos(point[theta]);
    point[theta] = range * std::sin(point[theta]);
  }
}

template <typename T>
void
polarToCartesian(T* point)
{
  constexpr int r = 0;
  constexpr int theta = 1;

  T range = point[r];
  point[0] = range * std::cos(point[theta]); // x
  point[1] = range * std::sin(point[theta]); // y
}

template <typename T>
void
cartesianToPolar(std::vector<std::array<T, 2>>& landmarks_in_scan_frame)
{
  constexpr int x = 0;
  constexpr int y = 1;
  for(auto& landmark: landmarks_in_scan_frame)
  {
    auto theta = std::atan2(landmark[y], landmark[x]);
    landmark[0] = std::sqrt(landmark[x]*landmark[x] + landmark[y]*landmark[y]);
    landmark[1] = theta;
  }
}

template <typename T>
void cartesianToPolar(T* point)
{
  constexpr int x = 0;
  constexpr int y = 1;

  auto theta = std::atan2(point[y], point[x]);
  point[0] = std::sqrt(point[x]*point[x] + point[y]*point[y]);
  point[1] = theta;
}


// copied form numpy doc 
// https://numpy.org/doc/stable/reference/generated/numpy.allclose.html
template <typename T, int m, int n>
bool allClose(const mat<T, m, n>& a, const mat<T, m, n>& b, 
  double rtol=1e-5, double atol=1e-8)
{
  return ((a-b).array().abs() <= (atol + rtol*b.array().abs())).all();
}


template <typename T>
tf2::Quaternion headingToQuaternion(T heading)
{
  tf2::Quaternion quat;
  quat.setRPY(0, 0, heading);
  quat.normalize();
  return quat;
}

template <typename T>
T 
quaternionToHeading(tf2::Quaternion const& quat)
{
  tf2::Matrix3x3 m(quat);
  
  T aux, yaw;
  m.getRPY(aux, aux, yaw);

  return yaw;
}


inline int
rowColIndexTo1D(int row, int col, int row_size)
{
  return row*row_size + col;
}

template <typename T, typename Derived>
void unsafeFillSparseWithDenseMatrix(spmat<T>& sparse_matrix, 
                                     Eigen::MatrixBase<Derived> const& dense_matrix,
                                     int row_offset=0,
                                     int col_offset=0)
{
  // this method is unsafe because it uses SparseMatrix::insert() instead of
  // SparseMatrix::coeffRef. Doc can be found in below link
  // https://eigen.tuxfamily.org/dox/group__TutorialSparse.html

  auto& sm = sparse_matrix;
  auto& dm = dense_matrix;

  for(int j = 0; j < dm.cols(); j++)
    for(int i = 0; i < dm.rows(); i++)
      sm.insert(i+row_offset, j+col_offset) = dm(i,j);
}

template <typename T1, typename T2>
bool intersect(T1 begin1, T1 end1, T2 begin2, T2 end2)
{
  while(begin1 != end1 && begin2 != end2)
  {
    if(*begin1 < *begin2)
    {
      begin1++;
      continue;
    }
    else if(*begin1 > *begin2)
    {
      begin2++;
      continue;
    }
    return true; // if is not greater os smaller, they are the same
  }

  return false;
}

template <int block_size=1>
spmatb permutationMatrix(std::vector<uint16_t> const& permutation)
{
  using Triplet = Eigen::Triplet<bool>;

  uint32_t dim = permutation.size() * block_size;
  std::vector<Triplet> triplets_list(dim);
  for(int i = 0; i < permutation.size(); i++)
    for(int j = 0; j < block_size; j++)
    {
      int row = i*block_size + j;
      int col = permutation[i]*block_size + j;
      triplets_list[row] = Triplet(row, col, 1);
    }
  
  spmatb p(dim, dim);
  p.setFromTriplets(triplets_list.begin(), triplets_list.end());
  return p;
}

// creates a matrix M of size dim x dim-1 blocks that merges (i.e. sums) 
// two columns of the A, when right multiplied A * M
template <int block_size=1>
spmatb mergeMatrix(uint16_t dim, uint16_t i, uint16_t j, int robot_dim=0)
{
  using Triplet = Eigen::Triplet<bool>;

  if(i > j)
    swap(i, j);

  std::vector<Triplet> triplets_list(dim);
  
  // robot block
  int k = 0;
  for(; k < robot_dim; k++)
    triplets_list[k] = Triplet(k, k, 1);

  int num_blocks = (dim - robot_dim) / block_size;
  int b = 0; 
  while(b < j)
  {
    for(int l = 0; l < block_size; l++)
    {
      int index = robot_dim + b*block_size + l;
      triplets_list[k++] = Triplet(index, index, 1);
    }
    b++;
  }
  // b == j
  for(int l = 0; l < block_size; l++)
  {
    int row = robot_dim + b*block_size + l;
    int col = robot_dim + i*block_size + l;
    triplets_list[k++] = Triplet(row, col, 1);
  }
  b++;

  // b > j
  int max_index = -1;
  while(b < num_blocks)
  {
    for(int l = 0; l < block_size; l++)
    {
      int index = robot_dim + b*block_size + l;
      triplets_list[k++] = Triplet(index, index-block_size, 1);
      max_index = index;
    }
    b++;
  }

  spmatb M(dim, dim-block_size);
  M.setFromTriplets(triplets_list.begin(), triplets_list.end());
  return M;
}

template <typename T>
void
sparseMatrixToTriplets(spmat<T> const& mat, 
  std::vector<Eigen::Triplet<T>>& triplets, int shift=0)
{
  triplets.reserve(triplets.size() + mat.nonZeros());

  for(int k=0; k < mat.outerSize(); k++)
    for(typename spmat<T>::InnerIterator it(mat, k); it; ++it)
      triplets.insert(triplets.end(), 
        Eigen::Triplet<T>(it.row()+shift, it.col()+shift, it.value()));
}

template <typename T>
std::vector<Eigen::Triplet<T>>
sparseMatrixToTriplets(spmat<T> const& mat)
{
  std::vector<Eigen::Triplet<T>> triplets;
  sparseMatrixToTriplets(mat, triplets);
  return triplets;
}

template <typename T, int block_size>
spmat<T>
constantBlockDiagonalMatrix(int num_blocks, 
  mat<T, block_size, block_size> const& block)
{
  using Triplet = Eigen::Triplet<T>;

  int dim = num_blocks * block_size;
  std::vector<Triplet> triplets_list( dim*block_size );

  int k = 0;
  for(int b = 0; b < num_blocks; b++)
    for(int j = 0; j < block_size; j++)
    {
      int col = b*block_size + j;
      for(int i = 0; i < block_size; i++)
      {
        int row = b*block_size + i;
        triplets_list[k++] = Triplet(row, col, block(i, j));
      }
    }
  
  spmat<T> diagonal_matrix(dim, dim);
  diagonal_matrix.setFromTriplets(triplets_list.begin(), triplets_list.end());

  return diagonal_matrix;
}

template <typename T>
spmat<T>
marginalizeInformationMatrix(spmat<T> const& information_matrix, 
                             spmat<bool> const& projection_matrix,
                             T epsilon,
                             mat<T>* inv_out=nullptr)
{
  spmat<T> proj = projection_matrix.cast<T>();
  auto proj_t = proj.transpose();
  
  auto& M = information_matrix;
  mat<T> inv = (proj * M * proj_t).toDense().inverse();

  if(inv_out != nullptr)
    *inv_out = inv;

  return (M - M*proj_t * inv * proj*M).pruned(epsilon, 1);
}

template <typename T>
void
marginalizeInformationMatrixAndVector(spmat<T>& information_matrix,
                                      vec<T>& information_vector,
                                      spmatb const& projection_matrix,
                                      T epsilon)
{
  mat<T> inv;
  spmat<T>& M = information_matrix; 
  information_matrix = marginalizeInformationMatrix(M, projection_matrix,
    epsilon, &inv);

  spmat<T> proj = projection_matrix.cast<T>();
  spmat<T> proj_t = proj.transpose();

  information_vector = information_vector
    - M*proj_t * inv * proj*information_vector;
}


matd<2>
fromMsg(utils::Matrix2D const& msg);

// create a square matrix M that when left multiplied by A, 
// removes some columns A
spmatb removeMatrix(int dim, std::vector<uint16_t> const& indexes);

template <typename T>
constexpr T
probability_to_log_odds(T p)
{
  return std::log(p / (1.0 - p));
}

template <typename T>
constexpr T
log_odds_to_probability(T log_odd)
{
  return 1.0 - (1.0 / (1.0 + std::exp(log_odd)));
}

} // end math

#endif
