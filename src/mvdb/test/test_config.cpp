#include <gtest/gtest.h>
#include "config.hxx"

TEST(test_config, load)
{
  using namespace mvdb;
  
  std::cerr << " come on give me a fucking breakpoint \n";
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/";
  ConfigReader cr { "/mnt/media/user/repos/lidar_vector_slam_refactor/" };

  auto local = cr.get_local_mapper_defaults();
  auto global = cr.get_global_mapper_defaults();
  auto proj = cr.get_projection_defaults();

  std::cerr << "projection.K = \n" << proj.K << "\n";
  std::cerr << "projection.h = " << proj.h << "\n";
}