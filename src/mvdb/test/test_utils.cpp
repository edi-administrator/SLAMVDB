#include <gtest/gtest.h>
#include <filesystem>
#include <sstream>
#include "utils.hxx"
#include "ros_utils.hxx"

TEST(test_io_utils, read_coord_success_size)
{
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/testdata/cloud_a.csv";
  auto coords = mvdb::coords_from_csv( target.string() );
  EXPECT_TRUE(coords.has_value());
  if (coords.has_value())
  {
    EXPECT_EQ(coords.value().size(), 11182);
  }
}


TEST(test_io_utils, read_vec_success_size)
{
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/testdata/cloud_a.csv";
  auto coords = mvdb::vectors_from_csv( target.string() );
  EXPECT_TRUE(coords.has_value());
  if (coords.has_value())
  {
    EXPECT_EQ(coords.value().size(), 11182);
  }
}

TEST(test_io_utils, read_pose_success_size)
{
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/testdata/poses.csv";
  auto coords = mvdb::poses_from_csv( target.string() );
  EXPECT_TRUE(coords.has_value());
  if (coords.has_value())
  {
    EXPECT_EQ(coords.value().size(), 11);
  }
}

TEST(test_io_utils, ros_msg_roundtrip)
{
  using namespace mvdb;
  auto current = std::filesystem::current_path();
  auto target = current / "../../src/mvdb/testdata/cloud_a.csv";
  auto coords = coords_from_csv( target.string() );

  auto pcd = pcd_from_coord( coords.value() );
  auto rountdrip = coord_from_pcd( pcd );

  EXPECT_TRUE(coords.has_value());
  for ( auto i = 0; i < coords.value().size(); i++ )
  {
    EXPECT_NEAR( coords.value()[i][0], rountdrip[i][0], 1e-4);
    EXPECT_NEAR( coords.value()[i][1], rountdrip[i][1], 1e-4);
    EXPECT_NEAR( coords.value()[i][2], rountdrip[i][2], 1e-4);
  }
}