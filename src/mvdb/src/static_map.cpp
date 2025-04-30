#include "utils.hxx"
#include "record.hxx"
#include "config.hxx"
#include "buffer_nodes.hxx"
#include <thread>

using namespace mvdb;
using namespace std::chrono_literals;


int main( int argc, char** argv )
{
  
  rclcpp::init( argc, argv );

  rclcpp::Node pub_node { "publisher_node" };
  auto pub = pub_node.create_publisher<sensor_msgs::msg::PointCloud2>( "/static_map_points", 10 );

  ConfigReader cfr {};

  auto params = cfr.get_static_map_params();
  auto sbmp = std::make_shared<Submap>( SubmapParams { .tree_params = params.tree_params } );

  auto pb = std::make_shared<PoseBuffer>();
  pb->load_from_dir( params.poses_path );

  std::vector<std::filesystem::path> files;

  for ( auto& file : std::filesystem::directory_iterator( params.points_path ) )
  {
    auto path = file.path();
    auto key = tokenize( path.filename().stem(), '_' ).front();
    if ( key == "points" && path.extension() == ".bin" )
    {
      files.push_back( file.path() );
    }
  }

  std::sort( files.begin(), files.end(), path_stamp_comp );

  SphericalConstraint filter_far ( vec_t::Zero(), 30.0 );
  PointCropConstriant filter_points( 2.0, -0.75, 1.0 );

  for ( auto& file : files )
  {
    auto stamp = stamp_from_path( file );
    if ( pb->in_range( stamp ) )
    {
      auto pts = filter_far.get_within( filter_points.get_within( coords_from_bin( file ) ) );
      Octree temp { params.tree_params };
      temp.insert_scan( pts, coord_t { 0, 0, 0 } );
      sbmp->insert_scan( temp.filtered_pts(), pb->pose_at( stamp ) );
      pub->publish( pcd_from_coord( sbmp->get_points_sbmp() ) );
    }
    if ( !rclcpp::ok() )
    {
      break;
    }
  }

  while ( rclcpp::ok() )
  {
    pub->publish( pcd_from_coord( sbmp->get_points_sbmp() ) );
    std::this_thread::sleep_for( 100ms );
  }

  rclcpp::shutdown();
  
}