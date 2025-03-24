#include <gtest/gtest.h>
#include <iostream>
#include "record.hxx"

TEST(record_test, constructors)
{
  using namespace mvdb;

  Eigen::Matrix<double, 100, 5> A = decltype(A)::Zero();
  Eigen::Vector<double, 5> n = A.colwise().norm();
  std::cerr << "colwise norm reduction result: " << n.transpose() << "\n";

  Octree tree; // Now MANDATORY for submap, because submaps are constructed only from occupancy octrees
  SubmapParams params { .tree_params = OctreeParams { .downsample = 1 } };
  Submap s { params };
  std::cerr << "constructed submap successfully" << std::endl;
  std::cerr << "tree = " << tree << std::endl;
}