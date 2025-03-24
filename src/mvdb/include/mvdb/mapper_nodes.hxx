#pragma once

#include <exception>
#include <queue>
#include <unordered_set>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "mvdb_interface/msg/loop_message.hpp"
#include "mvdb_interface/srv/map_query_discrete.hpp"
#include "mvdb_interface/srv/map_query_discrete_planar.hpp"
#include "mvdb_interface/srv/map_query_semantic.hpp"
#include "mvdb_interface/srv/map_query_spatial.hpp"
#include "buffer_nodes.hxx"
#include "record.hxx"
#include "projection.hxx"

namespace mvdb
{

inline bool default_split_condition( const std::vector<pose_t>& scan_poses )
{
  return scan_poses.size() > 20 && path_length( scan_poses ) > 30 && delta_t( scan_poses.front(), scan_poses.back() ) > 20;
}

inline size_t default_split_point( const std::vector<pose_t>& scan_poses )
{
  if ( scan_poses.size() > 30 )
  {
    return scan_poses.size() - 11;
  }
  else
  {
    return size_t ( 0.75 * double(scan_poses.size()) );
  }
}

struct GlobalMapperParams
{
  std::chrono::milliseconds correction_period = 1000ms;
  std::string topic = "/mvdb_mapper/mapper_node/loops";
  std::shared_ptr<PoseBuffer> pbuf = nullptr;
  std::shared_ptr<PoseBuffer> tbuf = nullptr;
  rclcpp::NodeOptions opt = rclcpp::NodeOptions();
  ProjectionParams projection_params = ProjectionParams {};
  OctreeParams tree_params = OctreeParams { .maxdepth = 8, .res = 64 };
  LocalVoxelLookupParams lookup_params = LocalVoxelLookupParams { .tree_params = OctreeParams { .maxdepth = 16, .res = .25 } };
  GlobalVoxelMapParams global_vox_params = GlobalVoxelMapParams {};
};

class GlobalMapper : public rclcpp::Node
{
  using spatial_srv_t = mvdb_interface::srv::MapQuerySpatial;
  using semantic_srv_t = mvdb_interface::srv::MapQuerySemantic;
  using discrete_srv_t = mvdb_interface::srv::MapQueryDiscrete;
  using planar_srv_t = mvdb_interface::srv::MapQueryDiscretePlanar;

  using sem_img_t = std::shared_ptr<std::vector<sem_t>>;
  using sbmp_t = std::shared_ptr<Submap>;
  using pmap_t = std::map<size_t, pose_t>;
  using smap_t = std::map<size_t, sem_img_t>;
  using sset_t = std::unordered_set<sbmp_t>;

  public:

    GlobalMapper( const GlobalMapperParams& params );
    void push_submap( sbmp_t submap, const pmap_t& unused_poses, const smap_t& unused_images );
    void timer_cb_single();
    void timer_cb_correction();
    void loop_cb( mvdb_interface::msg::LoopMessage::UniquePtr msg );
    void render_submap( sbmp_t sbmp, bool insert = false );

    void semantic_cb( const std::shared_ptr<semantic_srv_t::Request> req, std::shared_ptr<semantic_srv_t::Response> resp );
    void discrete_cb( const std::shared_ptr<discrete_srv_t::Request> req, std::shared_ptr<discrete_srv_t::Response> resp );
    void discrete_planar_cb( const std::shared_ptr<planar_srv_t::Request> req, std::shared_ptr<planar_srv_t::Response> resp );
    void spatial_cb( const std::shared_ptr<spatial_srv_t::Request> req, std::shared_ptr<spatial_srv_t::Response> resp );


    std::vector<TimeInterval> remove_submap( sbmp_t sbmp );
    std::vector<coord_t> search( const sem_t& query, sem_t::Scalar threshold = 0.7, const SpatialConstraint& filter = IdentityConstraint() );

  protected:

    pose_t _extrapolate_pose_at( size_t timestamp_ns );

    std::shared_ptr<PoseBuffer> m_pbuf, m_ptbuf;
    MutablePointCloud2 m_pcd;
    std::shared_ptr<MutablePointCloud2> m_sem_pcd;
    size_t m_count = 0UL;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_occ_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_sem_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_grid_pub;
    rclcpp::Subscription<mvdb_interface::msg::LoopMessage>::SharedPtr m_loop_sub;

    OctreeParams m_tree_params;
    ProjectionParams m_projection_params;

    mutable std::mutex m_mutex_maps, m_mutex_queue, m_mutex_loops;
    std::queue<std::tuple<sbmp_t, pmap_t, smap_t>> m_queue;
    std::queue<std::tuple<size_t, size_t, pose_t>> m_loops;
    std::atomic<size_t> m_last_loop, m_last_map_end;

    rclcpp::TimerBase::SharedPtr m_background_timer_once;
    rclcpp::TimerBase::SharedPtr m_background_timer_corrections;

    rclcpp::CallbackGroup::SharedPtr m_cb_group_timer;
    rclcpp::CallbackGroup::SharedPtr m_cb_group_subscriber;

    std::unordered_map<size_t, sbmp_t> m_submaps_all;
    std::unordered_map<size_t, pose_t> m_submap_last_render_occ;
    std::unordered_map<size_t, pose_t> m_submap_last_render_sem;
    DisjointIntervalLookup<sbmp_t> m_submaps_by_time;
    std::unordered_map<size_t, std::shared_ptr<VoxelMapVisualizer>> m_visualizers;

    rclcpp::Service<spatial_srv_t>::SharedPtr m_srv_spatial;
    rclcpp::Service<semantic_srv_t>::SharedPtr m_srv_semantic;
    rclcpp::Service<discrete_srv_t>::SharedPtr m_srv_discrete;
    rclcpp::Service<planar_srv_t>::SharedPtr m_srv_planar;

    VoxelLookup m_voxel_lookup;
    GlobalVoxelMap m_global_voxel;

};

struct LocalMapperParams
{
  std::shared_ptr<ImageBuffer> ibuf = nullptr;
  std::shared_ptr<PoseBuffer> pbuf = nullptr;
  std::shared_ptr<ScanBuffer> sbuf = nullptr;
  std::shared_ptr<GlobalMapper> gmap = nullptr;
  std::function<bool(const std::vector<pose_t>&)> split_condition = default_split_condition;
  std::function<size_t(const std::vector<pose_t>&)> split_point = default_split_point;
  ProjectionParams projection_params = ProjectionParams {};
  OctreeParams tree_params = OctreeParams { .res=.25, .downsample=0 };
  rclcpp::NodeOptions opt = rclcpp::NodeOptions {};
  bool use_quantizer = false;
  std::shared_ptr<Quantizer> quantizer = nullptr;
};

class LocalMapper : public rclcpp::Node
{
  public:
    LocalMapper( const LocalMapperParams& params );

    void timer_cb();
    void publish_sem();
    inline Submap& map() { return *m_sbmp; } // for debugging and testing

  protected:

    std::function<bool(const std::vector<pose_t>&)> m_split_condition;
    std::function<size_t(const std::vector<pose_t>&)> m_split_point;

    std::shared_ptr<Quantizer> m_quantizer;
    
    std::vector<pose_t> m_scan_poses;
    std::vector<size_t> m_scan_stamps;
    std::map<size_t, std::shared_ptr<std::vector<sem_t>>> m_unused_images;
    std::map<size_t, pose_t> m_unused_poses;

    ProjectionParams m_proj_params;

    size_t m_ns_last;
    size_t m_ns_period;
    size_t m_ns_last_split;

    // Submap m_sbmp;
    std::shared_ptr<Submap> m_sbmp;

    std::shared_ptr<ImageBuffer> m_ibuf;
    std::shared_ptr<PoseBuffer> m_pbuf;
    std::shared_ptr<ScanBuffer> m_sbuf;
    std::shared_ptr<GlobalMapper> m_gmap;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_occ_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_sem_pub;
    rclcpp::TimerBase::SharedPtr m_timer;
};

}