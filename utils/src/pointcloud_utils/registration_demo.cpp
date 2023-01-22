#include <pointcloud_utils/registration_pipeline.hpp>
#include <ros/ros.h>

using namespace pointcloud;

void getClouds(PointCloud& source_cloud, PointCloud& target_cloud)
{
  source_cloud = readPointCloudFromFile("/home/wellington/temp/testeFolder/rb1.xyz");
  target_cloud = readPointCloudFromFile("/home/wellington/temp/testeFolder/rb2.xyz");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_registration_demo");

  PointCloud source, target;
  getClouds(source, target);

  registration::PipelineParameters params;

  registration::RegistrationPipeline pipeline(params);

  auto best_transform = pipeline.evaluate(source, target);

  std::cout << "Source size: " << source.size() << std::endl;
  std::cout << "Target size: " << target.size() << std::endl;

  std::cout << "Score: " << std::get<1>(best_transform) << std::endl;
  std::cout << "Transform:" << std::endl;
  std::cout << std::get<0>(best_transform) << std::endl;
}