#pragma once

#include <vector>
#include <algorithm>
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "typedefs.hxx"
#include "math_utils.hxx"
#include "record.hxx"

namespace mvdb
{

size_t stamp_to_ns( const builtin_interfaces::msg::Time& stamp );

size_t header_to_ns( const std_msgs::msg::Header& h );

builtin_interfaces::msg::Time ns_to_stamp( size_t timestamp_ns );

std_msgs::msg::Header ns_to_header( size_t timestamp_ns );

std::vector<coord_t> coord_from_pcd( const sensor_msgs::msg::PointCloud2& pcd, const SpatialConstraint& filter = IdentityConstraint() );

// sensor_msgs::msg::PointCloud2 pcd_from_coord( const std::vector<coord_t>& pts, size_t ns = 0, const std::string& parent = "world", const std::vector<double> occ = {} );
sensor_msgs::msg::PointCloud2 pcd_from_coord( const std::vector<coord_t>& pts, const std::string& parent = "world", bool use_color = false, const std::vector<double> occ = {} );

pose_t pose_from_msg( const geometry_msgs::msg::PoseStamped& msg );

pose_t pose_from_transform_msg( const geometry_msgs::msg::TransformStamped& msg );

geometry_msgs::msg::PoseStamped msg_from_pose( const pose_t& pose, size_t stamp = 0 );

geometry_msgs::msg::TransformStamped transform_msg_from_pose( const pose_t& pose, size_t stamp = 0 );

void splice_pointcloud_msgs( sensor_msgs::msg::PointCloud2& inout, const sensor_msgs::msg::PointCloud2& other );

class MutablePointCloud2
{
  public:
    MutablePointCloud2( bool has_color = false, const std::string& parent = "mapper_frame" );

    size_t put_incr( const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& color = {} );
    size_t put( size_t count, const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& color = {} );
    
    void update_pose( size_t index, const pose_t& pose );
    void reinsert( size_t index, const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& color = {} );
    void remove( size_t index );

    inline const sensor_msgs::msg::PointCloud2 msg() const { return *m_msg; }
    inline bool has_idx( size_t idx ) const { return m_index_map_points.find(idx) != m_index_map_points.end(); }

  protected:
    bool m_has_color;
    size_t m_count;
    std::unique_ptr<sensor_msgs::msg::PointCloud2> m_msg;
    std::vector<coord_t> m_points;
    std::vector<double> m_colors;
    std::map<size_t, std::pair<size_t,size_t>> m_index_map_points;
    std::map<size_t, std::pair<size_t,size_t>> m_index_map_bytes;
    std::map<size_t, pose_t> m_transforms_current;
    std::map<size_t, pose_t> m_transforms_last;
};

template<typename T>
inline std::vector<T> splice_vector( const std::vector<T>& a, const std::vector<T>& b, size_t start, size_t stop )
{
  std::vector<T> splice_vector { a.begin(), a.begin() + start };
  splice_vector.insert( splice_vector.end(), b.begin(), b.end() );
  splice_vector.insert( splice_vector.end(), a.begin() + stop, a.end() );
  return splice_vector;
}

class VoxelMapVisualizer
{
  public:
    VoxelMapVisualizer( std::shared_ptr<VoxelLookup> lookup, std::shared_ptr<MutablePointCloud2> pcd );
    ~VoxelMapVisualizer() { remove(); }
    inline void mark_should_reinsert() { m_should_reinsert = true; };
    void update_points();
    void reinsert();
    void remove();

  protected:

    std::vector<coord_t> m_spoints;
    std::vector<double> m_scolors;
    bool m_should_reinsert;
    std::shared_ptr<VoxelLookup> m_map;
    std::shared_ptr<MutablePointCloud2> m_pcd;
};

};
