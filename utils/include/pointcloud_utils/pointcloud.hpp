#ifndef __POINTCLOUD_HPP_
#define __POINTCLOUD_HPP_

#include <pointcloud_utils/feature_descriptor.hpp>

#include <random>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <fstream>

namespace pointcloud
{

inline std::random_device device;
inline std::mt19937 rng(device());

using Point = Eigen::Vector2d;
typedef std::vector<Point> PointCloud;

typedef FeatureDescriptor<8> Descriptor;

typedef std::vector<FeatureDescriptor<8>> DescriptorsCloud;

typedef std::vector<std::pair<int,int>> Matches;

struct Neighbor
{
  int index;
  double distance;
};


template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream &ss, std::array<T, N> const&v)
{
  if(N == 0)
    return ss << "[]";

  ss << "[";
  for(int i = 0; i < N-1; i++)
    ss << v[i] << ", ";
  ss << v[N-1] << "]";

  return ss;
}

/**
 * returns vector of nearest neighbors of a given point in descending order
 */
std::vector<Neighbor> NearestNeighbors(PointCloud const& cloud, int index);

/**
 * returns the point nearest neighbor in the cloud
 */
Neighbor NearestNeighbor(PointCloud const& cloud, Point const& point);

PointCloud readPointCloudFromFile(std::string const& filename);

DescriptorsCloud getDescriptorsCloud(PointCloud const& cloud,
  uint8_t max_neighbors, double max_radius);

Matches matchDescriptors(DescriptorsCloud const& source,
  DescriptorsCloud const& target, double max_match_distance);

std::array<int, 2> selectMatchesRandomly(Matches const& matches);

Point getCentroid(PointCloud const& cloud);

Eigen::MatrixXd getCentralizedCloud(PointCloud const& cloud,
  Point const& centroid);

Eigen::Matrix3d getOptimalTransform(PointCloud const& source,
  PointCloud const&  target);

PointCloud transformCloud(PointCloud const& cloud,
  Eigen::Matrix3d const& transform);

Matches getInliers(PointCloud const& source_cloud, 
  PointCloud const& target_cloud, Eigen::Matrix3d transform);

std::tuple<Eigen::Matrix3d, size_t, double> RANSACTransformEstimation(
  PointCloud const& source, PointCloud const& target, Matches const& matches,
  uint32_t max_iterations, double inlier_threshold);

}
#endif