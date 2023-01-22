#include <pointcloud_utils/pointcloud.hpp>
#include <ros/ros.h>
#include <algorithm>

namespace pointcloud
{

std::vector<Neighbor> NearestNeighbors(PointCloud const& cloud, int index)
{
  int n = cloud.size();

  std::vector<Neighbor> neighbors;
  for(int i = 0; i < n; i++)
  {
    if(i != index)
    {
      Neighbor neighbor;
      neighbor.index = i;
      neighbor.distance = (cloud[index] - cloud[i]).norm();
      neighbors.push_back(neighbor);
    }
  }
  std::sort(neighbors.begin(), neighbors.end(),
    [](Neighbor const& a, Neighbor const& b){return a.distance < b.distance;});
  return neighbors;
}

Neighbor NearestNeighbor(PointCloud const& cloud, Point const& point)
{
  int n = cloud.size();

  Neighbor nn;
  nn.distance = std::numeric_limits<double>::max();
  for(int i = 0; i < n; i++)
  {
    double distance = (cloud[i] - point).norm();
    if(distance < nn.distance)
    {
      nn.index = i;
      nn.distance = distance;
    }
  }

  return nn;
}

PointCloud readPointCloudFromFile(std::string const& filename)
{
  PointCloud cloud;

  std::ifstream file(filename);
  double x, y;
  while(file >> x >> y)
    cloud.emplace_back(x,y);

  return cloud;
}

DescriptorsCloud getDescriptorsCloud(PointCloud const& cloud,
                                     uint8_t max_neighbors,
                                     double max_radius)
{
  DescriptorsCloud descriptorsCloud;
  int n = cloud.size();
  for(int i = 0; i < n; i++)
  {
    auto neighbors = NearestNeighbors(cloud, i);
    Descriptor fd;
    
    uint8_t n = neighbors.size() < max_neighbors
      ? neighbors.size() : max_neighbors;

    for(uint8_t j = 0; j < n; j++)
    {
      if(neighbors[j].distance > max_radius)
        break;

      int nn = neighbors[j].index;
      Eigen::Vector2d delta = cloud[nn] - cloud[i];
      fd.insert(atan2(delta[1], delta[0]));
    }
    descriptorsCloud.push_back(fd);
  }

  return descriptorsCloud;
}

Matches matchDescriptors(DescriptorsCloud const& source,
  DescriptorsCloud const& target, double max_match_distance)
{
  Matches matches;

  for(int s = 0; s < source.size(); s++)
  {
    int closest_index;
    double closest_squared_distance = std::numeric_limits<double>::max();
    for(int t = 0; t < target.size(); t++)
    {
      double squared_distance = (source[s] - target[t]).squared();
      if(squared_distance < closest_squared_distance)
      {
        closest_index = t;
        closest_squared_distance = squared_distance;
      }
    }

    if(std::pow(closest_squared_distance, 0.5) < max_match_distance)
      matches.push_back({s, closest_index});
  }
  return matches;
}

std::array<int, 2> selectMatchesRandomly(Matches const& matches)
{
  constexpr int S = 2;
  std::uniform_int_distribution<std::mt19937::result_type> dist(0, matches.size()-1);

  std::array<int, S> selected;
  for(int i = 0; i < S; i++)
  {
    int candidate;
    bool repeated;
    do
    {
      candidate = dist(rng);

      repeated = false;
      for(int j = 0; j < i; j++)
        if(candidate == selected[j])
          repeated = true;

    } while (repeated == true);
    
    selected[i] = candidate;
  }

  return selected;
}

Point getCentroid(PointCloud const& cloud)
{
  Eigen::Vector2d centroid(0,0);

  for(const auto& point : cloud)
    centroid += point;
  
  return centroid / cloud.size();
}

Eigen::MatrixXd getCentralizedCloud(PointCloud const& cloud,
  Point const& centroid)
{
  Eigen::MatrixXd centralized_cloud(cloud.size(), 2);

  for(int i = 0; i < cloud.size(); i++)
    centralized_cloud.block<1,2>(i, 0) = cloud[i] - centroid;
  
  return centralized_cloud;
}

Eigen::Matrix3d getOptimalTransform(PointCloud const& source,
  PointCloud const&  target)
{
  Eigen::Vector2d source_centroid = getCentroid(source);
  Eigen::Vector2d target_centroid = getCentroid(target);

  Eigen::MatrixXd centralized_source = getCentralizedCloud(source, source_centroid);
  Eigen::MatrixXd centralized_target = getCentralizedCloud(target, target_centroid);

  Eigen::MatrixXd H = centralized_source.transpose() * centralized_target.transpose();

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::MatrixXd U = svd.matrixU();

  Eigen::Matrix2d R = V * U.transpose();
  if(R.determinant() < 0)
  {
    ROS_DEBUG_STREAM("Transform calculation error");
    return Eigen::Matrix3d::Identity();
  }
  
  Eigen::Vector2d translation = target_centroid  - R * source_centroid;

  Eigen::Matrix3d transform = Eigen::Matrix3d::Identity();

  transform.block<2,2>(0,0) = R;
  transform.block<2,1>(0, 2) = translation;

  return transform;
}

PointCloud transformCloud(PointCloud const& cloud, Eigen::Matrix3d const& transform)
{
  PointCloud transformed_cloud(cloud.size());

  std::transform(cloud.begin(), cloud.end(), transformed_cloud.begin(),
    [&transform](Point const& point){
      Eigen::Vector3d p(point[0], point[1], 1);
      Eigen::Vector3d transformed = transform * p;
      return Point(transformed[0] / transformed[2],
        transformed[1] / transformed[2]);
    });

  return transformed_cloud;
}

Matches getInliers(PointCloud const& source_cloud, PointCloud const& target_cloud,
  Eigen::Matrix3d transform, double threshold, double& inliers_distance_sum)
{
  inliers_distance_sum = 0;
  PointCloud source_transformed = transformCloud(source_cloud, transform);

  Matches matches;
  for(int i = 0; i < source_transformed.size(); i++)
  {
    Point const& point = source_transformed[i];
    Neighbor nearest_neighbor = NearestNeighbor(target_cloud, point);
    if(nearest_neighbor.distance < threshold)
    {
      matches.push_back({i, nearest_neighbor.index});
      inliers_distance_sum += nearest_neighbor.distance;
    }
  }

  return matches;
}

std::tuple<Eigen::Matrix3d, size_t, double> RANSACTransformEstimation(
  PointCloud const& source, PointCloud const& target,
  Matches const& matches, uint32_t max_iterations, double inlier_threshold)
{
  std::tuple<Eigen::Matrix3d, size_t, double> best;
  std::get<1>(best) = 0;
  for(int i = 0; i < max_iterations; i++)
  {
    auto selected_matches = selectMatchesRandomly(matches);

    PointCloud source_sample, target_sample;
    for(auto const& selected : selected_matches)
    {
      source_sample.push_back(source[matches[selected].first]);
      target_sample.push_back(target[matches[selected].second]);
    }
    Eigen::Matrix3d transform = getOptimalTransform(source_sample, target_sample);

    double inliers_sum;
    auto inliers = getInliers(source, target, transform, inlier_threshold,
      inliers_sum);

    if(inliers.size() > std::get<1>(best))
    {
      std::get<1>(best) = inliers.size();
      std::get<0>(best) = transform;
      std::get<2>(best) = inliers_sum;

    }
    else if(inliers.size() == std::get<1>(best))
    {
      if(inliers_sum < std::get<2>(best))
      {
        std::get<1>(best) = inliers.size();
        std::get<0>(best) = transform;
        std::get<2>(best) = inliers_sum;
      }
    }
  }

  return best;
}

}