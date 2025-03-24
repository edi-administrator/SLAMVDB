#include "buffer_nodes.hxx"

// TEMPORARY
#include "visualization.hxx"
// TEMPORARY

namespace mvdb
{

constexpr const char* k_dtype_spec ("torch.float32");

template<typename T_value>
ThreadsafeStampMap<T_value>& ThreadsafeStampMap<T_value>::operator=( const std::map<size_t, T_value>& other )
{
  auto l = _get_lock();
  m_map = other;
  return *this;
}

template<typename T_value>
std::map<size_t, T_value> ThreadsafeStampMap<T_value>::get_all_copy() const
{
  auto l = _get_lock();
  return std::map<size_t, T_value> { m_map.begin(), m_map.end() };
}

template<typename T_value>
ThreadsafeStampMap<T_value>& ThreadsafeStampMap<T_value>::operator=( const ThreadsafeStampMap<T_value>& other )
{
  if ( this != &other )
  {
    *this = other.get_all_copy();
  }
  return *this;
}

template<typename T_value>
ThreadsafeStampMap<T_value>::ThreadsafeStampMap( const std::map<size_t, T_value>& other )
 : m_map(other)
{
};

template<typename T_value>
ThreadsafeStampMap<T_value>::ThreadsafeStampMap( const ThreadsafeStampMap<T_value>& other )
{
  m_map = other.get_all_copy();
}

template<typename T_value>
std::pair<
  typename std::map<size_t, T_value>::iterator, 
  typename std::map<size_t, T_value>::iterator>
ThreadsafeStampMap<T_value>::get_bounds( size_t start, size_t stop )
{
  auto l = _get_lock();
  return _get_bounds( start, stop, m_map );
}

template<typename T_value>
std::pair<
  std::pair<size_t,T_value>, 
  std::pair<size_t,T_value>> 
ThreadsafeStampMap<T_value>::get_boundary_kv( size_t start, size_t stop )
{
  auto l = _get_lock();
  auto bounds = _get_bounds( start, stop, m_map );

  size_t t_start = bounds.first->first;
  size_t t_end = bounds.second->first;

  T_value v_start = bounds.first->second;
  T_value v_end = bounds.second->second;

  std::pair<size_t, T_value> kv_first { t_start, v_start };
  std::pair<size_t, T_value> kv_second { t_end, v_end };

  return { kv_first, kv_second };
}

template<typename T_value>
std::map<size_t, T_value>
ThreadsafeStampMap<T_value>::get_range( size_t start, size_t stop, bool erase )
{
  auto bounds = get_bounds( start, stop );
  auto l = _get_lock();
  std::map<size_t, T_value> out { bounds.first, bounds.second };
  if ( erase )
  {
    m_map.erase( bounds.first, bounds.second );
  }
  return out;
}

template<typename T_value>
std::map<size_t, T_value>
ThreadsafeStampMap<T_value>::pull_range( size_t start, size_t stop )
{
  return get_range( start, stop, true );
}

template<typename T_value>
void ThreadsafeStampMap<T_value>::put( size_t key, const T_value& value )
{
  auto l = _get_lock();
  m_map[key] = value;
}

template<typename T_value>
void ThreadsafeStampMap<T_value>::put_all( const std::map<size_t, T_value>& other )
{
  auto l = _get_lock();
  m_map.insert( other.begin(), other.end() );
}


ImageBuffer::ImageBuffer( const ImageBufferParams& params )
: rclcpp::Node( params.node_name, params.node_options )
{

  m_cb_group_srv = this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
  m_cb_group_sub = this->create_callback_group( rclcpp::CallbackGroupType::Reentrant );

  rclcpp::SubscriptionOptions subopt;
  subopt.callback_group = m_cb_group_sub;

  m_in_progress.store(false);

  m_image_sub = create_subscription<sensor_msgs::msg::Image>( 
    params.topic, 10, std::bind( &ImageBuffer::image_cb, this, std::placeholders::_1 ), subopt
  );

  m_client = create_client<segserv_t>( params.service_path, rmw_qos_profile_services_default, m_cb_group_srv );
  while (!m_client->wait_for_service(1s) && rclcpp::ok() )
  {
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(get_logger(), "node up!");

}

std::map<size_t, std::shared_ptr<std::vector<ImageBuffer::elem_t>>> ImageBuffer::pull_range( size_t stamp_start, size_t stamp_end )
{
  // RCLCPP_INFO( get_logger(), "stamp_end - stamp_start %lu", stamp_end - stamp_start );
  return m_stamped_vimg_.pull_range( stamp_start, stamp_end );
}

std::tuple<size_t,size_t,size_t> ImageBuffer::h_w_c( size_t timestamp_ns )
{
  auto bounds = m_stamped_sizes_.get_boundary_kv( timestamp_ns, timestamp_ns );
  return bounds.first.second;
}

void ImageBuffer::image_cb( sensor_msgs::msg::Image::UniquePtr img )
{
  if ( m_in_progress.load() )
  {
    // RCLCPP_INFO(get_logger(), "service call already made...");
  }
  else
  {
    std::unique_lock<std::mutex> lock ( m_mutex_service, std::try_to_lock );

    if ( lock.owns_lock() )
    {
      if ( m_in_progress.load() )
      {
        // RCLCPP_INFO(get_logger(), "race condition averted! service call already made...");
      }
      else
      {
        m_in_progress.store(true);

        auto timestamp_ns = header_to_ns( img->header );
        // RCLCPP_INFO( get_logger(), "making service call, t=%lu", timestamp_ns );

        auto request = std::make_shared<segserv_t::Request>();
        request->img = *img;
        
        auto callback = [&, timestamp_ns]( client_t::SharedFutureWithRequest future )
        {
          auto req_resp_pair = future.get();
          auto resp = req_resp_pair.second;
          // RCLCPP_INFO(get_logger(), "future callback! result type %s h %d w %d c %d", resp->vimg.dtype.c_str(), resp->vimg.h, resp->vimg.w, resp->vimg.c );
          if ( resp->vimg.dtype == k_dtype_spec )
          {
            auto dptr = static_cast<void*>(resp->vimg.data.data());
            auto _fptr = static_cast<elem_t::Scalar*>(dptr);

            size_t step = resp->vimg.c;
            size_t size = resp->vimg.w * resp->vimg.h * resp->vimg.c;

            auto vimg = std::make_shared<std::vector<elem_t>> ();
            vimg->reserve( resp->vimg.h * resp->vimg.w );

            for ( auto fptr = _fptr; fptr <= _fptr + size - step; fptr += step )
            {
              elem_t px = Eigen::Map<elem_t>(fptr);
              vimg->push_back( px );
            }
            
            m_stamped_vimg_.put( timestamp_ns, vimg );
            m_stamped_sizes_.put( timestamp_ns, { resp->vimg.h, resp->vimg.w, resp->vimg.c } );

          }
          m_in_progress.store(false);
        };

        m_client->async_send_request(request, callback);
      }
    }
  }

}


ScanBuffer::ScanBuffer( const ScanBufferParams& params )
 : rclcpp::Node( params.node_name, params.node_options )
{
  m_lidar_sub = create_subscription<sensor_msgs::msg::PointCloud2>( 
    params.topic, 10, std::bind( &ScanBuffer::scan_cb, this, std::placeholders::_1 ) 
  );
}

void ScanBuffer::scan_cb( sensor_msgs::msg::PointCloud2::UniquePtr scan )
{
  auto timestamp_ns = header_to_ns( scan->header );
  auto coord = coord_from_pcd( *scan, OuterSphericalConstraint { vec_t::Zero(), 1.5 } );
  
  m_stamped_pts_.put( timestamp_ns, coord );
  m_last_stamp.store(timestamp_ns);
}

std::map<size_t, std::vector<coord_t>> ScanBuffer::get_up_to( size_t timestamp_ns )
{
  return m_stamped_pts_.pull_range( 0UL, timestamp_ns );
}

PoseBuffer::PoseBuffer( const PoseBufferParams& params  )
: rclcpp::Node( params.node_name, params.node_options )
{
  m_transform_sub = create_subscription<geometry_msgs::msg::TransformStamped>( 
    params.topic, 10, std::bind( &PoseBuffer::transform_cb, this, std::placeholders::_1 ) 
  );

  m_pts_pub = create_publisher<sensor_msgs::msg::PointCloud2>( params.node_name +"_poses", 1 );
}
    
void PoseBuffer::load_from_dir( const std::string& path, const std::string& ext )
{
  auto stamped_poses = stamped_poses_from_csvs( path, ext );
  for ( auto& [stamp, pose] : stamped_poses )
  {
    m_stamped_pose_.put(stamp, pose);
    if ( stamp > m_latest_stamp.load() )
    {
      m_latest_stamp.store(stamp);
    }
  }
}

void PoseBuffer::transform_cb( geometry_msgs::msg::TransformStamped::UniquePtr transform_stamped )
{
  pose_t T = pose_from_transform_msg( *transform_stamped );
  auto timestamp_ns = header_to_ns( transform_stamped->header );
  if ( timestamp_ns > last_stamp() )
  {
    m_latest_stamp.store(timestamp_ns);
  }
  m_stamped_pose_.put(timestamp_ns, T);
}

pose_t PoseBuffer::pose_at( size_t timestamp_ns )
{
  auto boundary_kv = m_stamped_pose_.get_boundary_kv( timestamp_ns, timestamp_ns );
  auto& [t_start, T_start] = boundary_kv.first;
  auto& [t_end, T_end] = boundary_kv.second;
  
  double t_a, t_b, t;
  t_a = 0;
  // t_b = double( t_end - t_start ) / 1e9;
  // t = double( timestamp_ns - t_start ) / 1e9;
  t_b = double( t_end - t_start );
  t = double( timestamp_ns - t_start );

  t = std::max( t, 0. );
  t = std::min( t, t_b );

  return interpolate_pose( T_start, T_end, t_a, t_b, t );
}

std::pair<size_t, pose_t> PoseBuffer::latest_at( size_t timestamp_ns )
{
  auto boundary_kv = m_stamped_pose_.get_boundary_kv( timestamp_ns, timestamp_ns );
  return boundary_kv.first;
}

std::vector<pose_t> PoseBuffer::get_all()
{
  std::vector<pose_t> out;
  auto map_copy = m_stamped_pose_.get_all_copy();
  
  for ( auto& [_,pose] : map_copy )
  {
    out.push_back(pose);
  }
  
  return out;
}

void PoseBuffer::publish_pose_cloud()
{

  auto poses = get_all();
  sensor_msgs::msg::PointCloud2 out;

  std::vector<coord_t> pts;
  std::vector<double> occ;

  for ( auto& pose : poses )
  {
    pose_cloud( pts, occ, pose );
  }

  m_pts_pub->publish( pcd_from_coord( pts, "mapper_frame", true, occ ) );
}


EmbeddingBuffer::EmbeddingBuffer( const std::string& srv_path, const rclcpp::NodeOptions& opt )
: rclcpp::Node( "embedding_buffer", opt )
{
  m_client = create_client<embserv_t>(srv_path);
  while ( !m_client->wait_for_service(1s) && rclcpp::ok() )
  {
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(get_logger(), "node up!");
}

EmbeddingBuffer::client_t::FutureAndRequestId EmbeddingBuffer::call_async_noshare( const std::string& s ) const
{
  auto req = std::make_shared<embserv_t::Request>();
  req->query = s;
  return m_client->async_send_request(req);
}

} // namespace mvdb