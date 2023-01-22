#include <pointcloud_utils/registration_pipeline.hpp>

namespace pointcloud
{

namespace registration
{

Result
RegistrationPipeline::evaluate(PointCloud const& source,
  PointCloud const& target) const
{
  auto source_descriptors = getDescriptorsCloud(source, _params.max_neighbors,
    _params.max_radius);
  auto target_descriptors = getDescriptorsCloud(target, _params.max_neighbors,
    _params.max_radius);

  Matches matches = matchDescriptors(source_descriptors, target_descriptors,
    _params.descriptors_match_max_distance);
  
  ROS_WARN_STREAM("matches size: " << matches.size());
  if(matches.size() < 2)
    return std::make_pair(false, math::mat3d());
  
  auto transform = RANSACTransformEstimation(source, target, matches,
    _params.ransac_max_iterations, _params.inlier_threshold);
  
  bool is_correct_transform = checkResult(source, target, transform);
  ROS_ERROR_STREAM("is correct?: " << is_correct_transform);
  return std::make_pair(is_correct_transform, std::get<0>(transform));
}

void 
RegistrationPipeline::evaluateResult(PointCloud const& cloud_a,
    PointCloud const& cloud_b, int& not_corrected_matched) const
{
  for(auto& point : cloud_a)
  {
    auto nearestNeighbor = NearestNeighbor(cloud_b, point);

    double d = nearestNeighbor.distance;
    if(d > _params.inlier_threshold && d < _params.fuzzy_area_threshold)
      not_corrected_matched += 1;
  }
}

bool
RegistrationPipeline::checkResult(PointCloud const& source,
  PointCloud const& target,
  std::tuple<math::mat3d, size_t, double> const& transform_data) const
{
  auto source_transformed = transformCloud(source, std::get<0>(transform_data));

  int not_corrected_matched = 0;
  evaluateResult(source_transformed, target, not_corrected_matched);
  evaluateResult(target, source_transformed, not_corrected_matched);

  auto inliers = std::get<1>(transform_data);
  double mean_error = std::get<2>(transform_data) / inliers;
  ROS_INFO_STREAM("not corrected matched: " << not_corrected_matched 
    << " mean error: " << mean_error);
  ROS_INFO_STREAM("inliers" << inliers);
  if(not_corrected_matched == 0 && mean_error < 2e-2 && inliers > 4)
    return true;
  
  return false;
}

}

}