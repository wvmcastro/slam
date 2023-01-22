#ifndef __REGISTRATION_PIPELINE_HPP_
#define __REGISTRATION_PIPELINE_HPP_

#include <math_utils/math.hpp>
#include <pointcloud_utils/pointcloud.hpp>

namespace pointcloud
{

namespace registration
{

using Result =  std::tuple<bool, math::mat3d>;

struct PipelineParameters
{
  uint32_t ransac_max_iterations = 500;
  double inlier_threshold = 0.05;
  double fuzzy_area_threshold = 0.7;
  uint8_t max_neighbors = 8;
  double max_radius = 1.5;
  double descriptors_match_max_distance = 2;
};

class RegistrationPipeline
{

public:
  RegistrationPipeline() = default;

  RegistrationPipeline(PipelineParameters const& params)
    : _params{params}
  {
    // do nothing
  }

  Result evaluate(PointCloud const& source, PointCloud const& target) const;
  
private:
  const PipelineParameters _params;
  
  bool checkResult(PointCloud const& source, PointCloud const& target,
    std::tuple<math::mat3d, size_t, double> const& transform_data) const;
  
  void evaluateResult(PointCloud const& cloud_a,
    PointCloud const& cloud_b, int& not_corrected_matched) const;
};


}
}

#endif