#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <type_traits>
#include <algorithm>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mvdb_interface/srv/segmentation.hpp"
#include "mvdb_interface/srv/embedding.hpp"


#include "typedefs.hxx"
#include "ros_utils.hxx"
#include "utils.hxx"


#include <chrono>
using namespace std::chrono_literals;

namespace mvdb
{

template<
  typename T_key, 
  typename T_value, 
  std::enable_if_t<std::is_arithmetic_v<T_key>, int> = 0>
std::pair<
  typename std::map<T_key, T_value>::iterator, 
  typename std::map<T_key, T_value>::iterator> 
_get_bounds( T_key start, T_key stop, std::map<T_key, T_value>& map_in )
{
  auto lbound = map_in.lower_bound(start);
  auto ubound = map_in.upper_bound(stop);

  if ( lbound != map_in.begin() ) 
  {
    lbound--;
  }

  return { lbound, ubound };
}

template<
  typename T_key, 
  typename T_value, 
  std::enable_if_t<std::is_arithmetic_v<T_key>, int> = 0
>
std::map<
  T_key, 
  T_value> 
_pull_range( T_key start, T_key stop, std::map<T_key, T_value>& map_in )
{
  auto bounds = _get_bounds<T_key, T_value>( start, stop, map_in );
  std::map<T_key, T_value> out { bounds.first, bounds.second };
  map_in.erase( bounds.first, bounds.second );
  return out;
}


template<typename T_value>
class ThreadsafeStampMap
{
  public:
    ThreadsafeStampMap() {};

    ThreadsafeStampMap( const std::map<size_t, T_value>& other );
    ThreadsafeStampMap& operator=( const std::map<size_t, T_value>& other );

    ThreadsafeStampMap( const ThreadsafeStampMap& other );
    ThreadsafeStampMap& operator=( const ThreadsafeStampMap& other );
    
    std::pair<
      typename std::map<size_t, T_value>::iterator, 
      typename std::map<size_t, T_value>::iterator>
    get_bounds( size_t start, size_t stop );

    std::pair<
      std::pair<size_t,T_value>, 
      std::pair<size_t,T_value>> 
    get_boundary_kv( size_t start, size_t stop );

    std::map<size_t, T_value>
    get_range( size_t start, size_t stop, bool erase = false );

    std::map<size_t, T_value>
    pull_range( size_t start, size_t stop );

    void put( size_t key, const T_value& value );

    void put_all( const std::map<size_t, T_value>& other );

    std::map<size_t, T_value> get_all_copy() const;

  protected:
    inline const std::unique_lock<std::mutex> _get_lock() const { return std::unique_lock<std::mutex>( m_mutex ); }
    mutable std::mutex m_mutex;
    std::map<size_t, T_value> m_map;
};

template class ThreadsafeStampMap<std::shared_ptr<std::vector<sem_t>>>;
template class ThreadsafeStampMap<pose_t>;
template class ThreadsafeStampMap<coord_t>;
template class ThreadsafeStampMap<std::tuple<size_t, size_t, size_t>>;

struct ImageBufferParams
{
  std::string node_name = "image_buffer";
  std::string topic = "/camera_left/image_raw";
  std::string service_path = "/segmentation";
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions {};
};

class ImageBuffer : public rclcpp::Node
{
  public:

    using segserv_t = mvdb_interface::srv::Segmentation;
    using client_t = rclcpp::Client<segserv_t>;
    using elem_t = sem_t;

    ImageBuffer( const ImageBufferParams& params = {} );

    void image_cb( sensor_msgs::msg::Image::UniquePtr img );
    std::map<size_t, std::shared_ptr<std::vector<elem_t>>> pull_range( size_t stamp_start, size_t stamp_end );
    std::tuple<size_t,size_t,size_t> h_w_c( size_t timestamp_ns );

    inline size_t size() const { return m_stamped_vimg.size(); }
    inline bool done() const { return !m_in_progress.load(); }

  protected:
    rclcpp::CallbackGroup::SharedPtr m_cb_group_sub;
    rclcpp::CallbackGroup::SharedPtr m_cb_group_srv;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_sub;
    client_t::SharedPtr m_client;

    std::atomic<bool> m_in_progress;
    std::mutex m_mutex_service, m_mutex_map;

    ThreadsafeStampMap<std::shared_ptr<std::vector<sem_t>>> m_stamped_vimg_;
    ThreadsafeStampMap<std::tuple<size_t,size_t,size_t>> m_stamped_sizes_;

    std::map<size_t, std::shared_ptr<std::vector<elem_t>>> m_stamped_vimg;
    std::map<size_t, std::tuple<size_t, size_t, size_t>> m_stamped_sizes;

};

struct PoseBufferParams
{
  std::string node_name = "tracker_pose_buffer";
  std::string topic = "/tracker_pose";
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions {};
};

class PoseBuffer : public rclcpp::Node
{
  public:
    PoseBuffer( const PoseBufferParams& params = {} );
    
    void load_from_dir( const std::string& path = "refactor_test_data", const std::string& ext = ".csv");
    void transform_cb( geometry_msgs::msg::TransformStamped::UniquePtr transform_stamped );
    pose_t pose_at( size_t timestamp_ns );
    std::pair<size_t, pose_t> latest_at( size_t timestamp_ns );

    std::vector<pose_t> get_all();
    void publish_pose_cloud();

    inline size_t last_stamp() const { return m_latest_stamp.load(); }
  
  protected:
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr m_transform_sub;
    ThreadsafeStampMap<pose_t> m_stamped_pose_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pts_pub;

    std::atomic<size_t> m_latest_stamp = 0UL;
};

struct ScanBufferParams
{
  std::string node_name = "scan_buffer_node";
  std::string topic = "/points";
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions {};
};

class ScanBuffer : public rclcpp::Node
{
  public:
    ScanBuffer( const ScanBufferParams& params = {} );

    void scan_cb( sensor_msgs::msg::PointCloud2::UniquePtr scan );
    inline size_t last_stamp() const { return m_last_stamp.load(); }
    std::map<size_t, std::vector<coord_t>> get_up_to( size_t timestamp_ns );

  protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    ThreadsafeStampMap<std::vector<coord_t>> m_stamped_pts_;
    std::atomic<size_t> m_last_stamp = 0;
};


class EmbeddingBuffer : public rclcpp::Node
{

  public:
    using embserv_t = mvdb_interface::srv::Embedding;
    using client_t = rclcpp::Client<embserv_t>;

    EmbeddingBuffer( const std::string& srv_path = "/embedding", const rclcpp::NodeOptions& opt = rclcpp::NodeOptions() );
    client_t::FutureAndRequestId call_async_noshare( const std::string& query ) const;

  protected:
    client_t::SharedPtr m_client;
};


}
