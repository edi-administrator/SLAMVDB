#include "mapper_nodes.hxx"

using namespace mvdb;


LocalMapper::LocalMapper( const LocalMapperParams& params )
: m_gmap( params.gmap ), 
  m_sbuf( params.sbuf ), 
  m_pbuf(params.pbuf), 
  m_ibuf( params.ibuf ), 
  m_quantizer( params.quantizer ),
  m_proj_params( params.projection_params ), 
  rclcpp::Node( "local_mapper", params.opt )
{

  m_split_condition = params.split_condition;
  m_split_point = params.split_point;

  SubmapParams sbmp_params {
    .tree_params = params.tree_params,
    .T_w_s = pose_t::Identity(),
  };

  if ( params.use_quantizer )
  {
    // Quantizing on insertion makes this slower. Don't do it unless needed for demo
    sbmp_params.quantizer = params.quantizer;
  }

  m_sbmp = std::make_shared<Submap>( sbmp_params );

  m_ns_last = 0UL;
  m_ns_last_split = 0UL;

  m_occ_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/local_mapper_occ", 1 );
  m_sem_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/local_mapper_sem", 1 );

  auto timer_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration(2000ms));
  m_timer = create_wall_timer( timer_duration, std::bind( &LocalMapper::timer_cb, this ) );
  m_ns_period = timer_duration.count();
}

void LocalMapper::timer_cb()
{
  auto image_process_padding = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration(500ms));
  auto image_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration(1000ms));

  size_t now = get_clock()->now().nanoseconds();
  size_t cycle_start = now;

  size_t t_last_scan = m_sbuf->last_stamp();
  size_t t_last_pose = m_pbuf->last_stamp();
  size_t t_last_item = std::min( t_last_scan, t_last_scan );

  size_t start = ( m_ns_last == 0 ) ? 0 : m_ns_last - image_process_padding.count() - image_delay.count();
  size_t end = ( t_last_item == 0 ) ? 0 : t_last_item - image_process_padding.count() - image_delay.count();

  auto scans_lately = m_sbuf->get_up_to( t_last_item - image_process_padding.count() );
  auto images_lately = m_ibuf->pull_range( start, end );

  m_ns_last = t_last_scan;

  RCLCPP_INFO( get_logger(), "pulled %lu scans", scans_lately.size() );
  RCLCPP_INFO( get_logger(), "pulled %lu images", images_lately.size() );

  auto scan_iterator = scans_lately.begin();
  auto img_iterator = images_lately.begin();

  while( now - cycle_start < m_ns_period / 2 - image_process_padding.count() && scan_iterator != scans_lately.end() )
  {
    auto& [stamp, scan] = *(scan_iterator++);

    pose_t T_w_scan = m_pbuf->pose_at(stamp);
    m_scan_poses.push_back( T_w_scan );
    m_scan_stamps.push_back( stamp );
    
    m_sbmp->clear_by_scan( scan, T_w_scan );
    
    now = get_clock()->now().nanoseconds();
  }

  while( now - cycle_start < m_ns_period - image_process_padding.count() &&  img_iterator != images_lately.end() )
  {
    auto& [stamp, image] = *(img_iterator++);

    pose_t T_w_scan = m_pbuf->pose_at(stamp);
    
    ProjectionBuffer pbuf { m_sbmp->pose().inverse() * T_w_scan, m_proj_params };
    pbuf.project( m_sbmp->tree() );
    pbuf.fill_sem( *image );
    
    m_sbmp->insert_new_records( pbuf.sem(), pbuf.counts(), pbuf.keys(), pbuf.times() );
    now = get_clock()->now().nanoseconds();
  }

  while( img_iterator != images_lately.end() )
  {
    auto& [stamp,image] = *(img_iterator++);
    m_unused_images[stamp] = image;
    m_unused_poses[stamp] = m_pbuf->pose_at(stamp);
  }

  if ( m_split_condition( m_scan_poses ) )
  {
    size_t split_point = m_split_point( m_scan_poses );
    std::vector<pose_t> repulse;
    std::vector<pose_t> attract;

    for ( size_t i = 0; i < split_point; i+=10 )
    {
      repulse.push_back( m_sbmp->pose().inverse() * m_scan_poses[i] );
    }
    for ( size_t i = split_point; i < m_scan_poses.size(); i+=10 )
    {
      attract.push_back( m_sbmp->pose().inverse() * m_scan_poses[i] );
    }

    VoronoiConstraint filter_keep { attract, repulse };

    pose_t grid_keep = m_scan_poses.back();
    pose_t grid_drop = m_scan_poses.front();

    size_t stamp_drop = m_scan_stamps.front();

    auto new_maps = m_sbmp->split_off( filter_keep, grid_keep, grid_drop );
    new_maps[1]->set_stamp( stamp_drop );
    new_maps[1]->add_interval( TimeInterval( m_ns_last_split + 1, m_ns_last ) ); // two issues: intervals aren't dense; this should fix
    m_ns_last_split = m_ns_last;

    m_sbmp = new_maps[0];

    m_scan_poses.clear();
    m_scan_stamps.clear();

    m_gmap->push_submap( new_maps[1], m_unused_poses, m_unused_images );
    m_unused_images.clear();
    m_unused_poses.clear();
  }

  auto pts = apply_T( m_sbmp->pose(),  m_sbmp->tree().filtered_pts( IdentityConstraint() ) );
  m_occ_pub->publish( pcd_from_coord( pts, "tracker_frame" ) );

  publish_sem();
}

void LocalMapper::publish_sem()
{
  auto keys = m_sbmp->filter_xkeys();
  auto pts = m_sbmp->tree().filtered_pts();

  std::vector<coord_t> coords;
  std::vector<double> color;

  for ( auto& key : keys )
  {
    auto record = m_sbmp->global_idx().at( m_sbmp->spatial_idx().at(key).value() );
    coords.push_back( apply_T( m_sbmp->pose(), m_sbmp->tree().xkey_to_coord(key) ) );
    color.push_back( m_quantizer->color_mono( record->similarities ) );
  }

  m_sem_pub->publish( pcd_from_coord( coords, "tracker_frame", true, color ) );
}

GlobalMapper::GlobalMapper( const GlobalMapperParams& params )
 : m_voxel_lookup( params.lookup_params ),
  m_global_voxel ( params.global_vox_params ),
  m_pcd(false), 
  m_sem_pcd( std::make_shared<MutablePointCloud2>( true ) ), 
  m_projection_params( params.projection_params ), 
  m_tree_params( params.tree_params ), 
  rclcpp::Node( "global_mapper", params.opt )
{

  m_occ_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/global_mapper_occ", 1 );
  m_sem_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/global_mapper_sem", 1 );
  m_grid_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/global_mapper_grid", 1 );

  if ( params.tbuf )
  {
    m_ptbuf = params.tbuf;    
  }
  else
  {
    throw std::out_of_range("no pointer to tracker pose buffer provided in global mapper!");
  }

  if ( params.pbuf )
  {
    m_pbuf = params.pbuf;    
  }
  else
  {
    throw std::out_of_range("no pointer to mapper pose buffer provided in global mapper!");
  }
  
  m_last_loop.store(0UL);

  m_cb_group_timer = this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );
  m_cb_group_subscriber = this->create_callback_group( rclcpp::CallbackGroupType::MutuallyExclusive );

  m_background_timer_once = this->create_wall_timer( 
    0ms, std::bind( &GlobalMapper::timer_cb_single, this ), m_cb_group_timer );
  m_background_timer_once->cancel();

  m_background_timer_corrections = this->create_wall_timer( 
    params.correction_period, std::bind( &GlobalMapper::timer_cb_correction, this ), m_cb_group_timer );

  rclcpp::SubscriptionOptions subopt;
  subopt.callback_group = m_cb_group_subscriber;
  m_loop_sub = this->create_subscription<mvdb_interface::msg::LoopMessage>( 
    params.topic, 10, std::bind( &GlobalMapper::loop_cb, this, std::placeholders::_1 ), subopt
  );

  m_srv_semantic = this->create_service<semantic_srv_t>( 
    "semantic_service",
    std::bind( &GlobalMapper::semantic_cb, this, std::placeholders::_1, std::placeholders::_2 ),
    rmw_qos_profile_services_default,
    m_cb_group_timer 
  );
}

void GlobalMapper::push_submap( sbmp_t submap, const pmap_t& unused_poses, const smap_t& unused_images )
{
  assert( unused_poses.size() == unused_images.size() );
  {
    std::lock_guard<std::mutex> lock_queue ( m_mutex_queue );
    m_queue.push( { submap, unused_poses, unused_images } );
    m_background_timer_once->reset();
  }
}


void GlobalMapper::semantic_cb( const std::shared_ptr<semantic_srv_t::Request> req, std::shared_ptr<semantic_srv_t::Response> resp )
{
  if ( req->dim == sem_t::RowsAtCompileTime )
  {
    RCLCPP_INFO( get_logger(), "[semantic search service] received request with dim = %u", req->dim );
    sem_t query = Eigen::Map<sem_t>( req->embedding.data() );
    auto results = search( query, req->similarity_thresh );
    resp->points = pcd_from_coord( results, "mapper_frame" );
    RCLCPP_INFO( get_logger(), "[semantic search service] returning response with size = %u", resp->points.width );
  }
  else
  {
    RCLCPP_ERROR( get_logger(), "[semantic search service] incorrect query vector size = %u", req->dim );
  }
}

std::vector<coord_t> GlobalMapper::search( const sem_t& query, sem_t::Scalar threshold, const SpatialConstraint& filter )
{
  auto predicate = [&query, threshold]( const Record& record ) -> bool
  {
    return cosine_similarity( record.value, query )[0] > threshold;
  };

  std::vector<coord_t> out;
  {
    std::lock_guard<std::mutex> lock_maps ( m_mutex_maps );
    for ( auto& cell : m_global_voxel.at_filtered( filter ) )
    {
      auto results = cell->get_all( predicate );
      for ( auto& record : results )
      {
        auto pt = cell->tree_params().xkey_to_coord_leaf( record.xkey );
        out.push_back( apply_T( cell->pose(), pt ) );
      }
    }
  }
  return out;
}

void GlobalMapper::loop_cb( mvdb_interface::msg::LoopMessage::UniquePtr msg )
{
  auto t_a = stamp_to_ns( msg->t_a );
  auto t_b = stamp_to_ns( msg->t_b );
  bool pushed = false;
  if ( t_a > m_last_loop.load() && t_b < m_last_map_end.load() && t_b < m_pbuf->last_stamp() ) // two issues: t_b is outside the range and the intervals aren't dense
  {
    std::lock_guard<std::mutex> lock_loops ( m_mutex_loops );
    if ( m_loops.empty() || t_a > std::get<0>( m_loops.back() ) )
    {
      m_last_loop.store( t_a );
      auto t_b = stamp_to_ns( msg->t_b );
      auto T_a_b = pose_from_transform_msg( msg->transform_a_b );
      m_loops.push( { t_a, t_b, T_a_b } );
      pushed = true;
    }
  }
  if ( pushed )
  {
    RCLCPP_INFO( get_logger(), "push loop at %lu", t_a );
  }
}

void GlobalMapper::timer_cb_correction()
{
  std::vector<std::tuple<size_t,size_t,pose_t>> loops;

  {
    std::lock_guard<std::mutex> lock_loops ( m_mutex_loops );
    while ( !m_loops.empty() )
    {
      loops.push_back( m_loops.front() );
      m_loops.pop();
    }
  }

  for ( auto& [idx, sbmp] : m_submaps_all )
  {
    sbmp->set_mapper_pose( _extrapolate_pose_at( sbmp->stamp() ) );
    render_submap( sbmp, false );
  }

  for ( auto& [t_target, t_source, _] : loops )
  {

    RCLCPP_INFO( get_logger(), "processing loop %lu -> %lu", t_source, t_target );

    auto opt_sbmp_target = m_submaps_by_time.get(t_target);
    auto opt_sbmp_source = m_submaps_by_time.get(t_source);

    RCLCPP_INFO( get_logger(), "has value: source %d target %d", opt_sbmp_source.has_value(), opt_sbmp_target.has_value() );

    if ( opt_sbmp_target.has_value() && opt_sbmp_source.has_value() )
    {
      auto& [interval_target, sbmp_target] = opt_sbmp_target.value();
      auto& [interval_source, sbmp_source] = opt_sbmp_source.value();

      auto xkey_source = m_tree_params.vec_to_xkey_leaf_centered( t_from_T( sbmp_source->mapper_pose() ) );
      auto xkey_target = m_tree_params.vec_to_xkey_leaf_centered( t_from_T( sbmp_target->mapper_pose() ) );

      RCLCPP_INFO( get_logger(), "xkey: source %lu target %lu", xkey_source, xkey_target );
      RCLCPP_INFO( get_logger(), "seq : source %lu target %lu", sbmp_source->seq_id(), sbmp_target->seq_id() );
      RCLCPP_INFO( get_logger(), "ptr : source %lu target %lu", size_t(sbmp_source.get()), size_t(sbmp_target.get()) );

      if ( sbmp_target != sbmp_source && xkey_source == xkey_target )
      {
        for ( auto& pulled_interval : remove_submap( sbmp_source ) )
        {
          sbmp_target->add_interval( pulled_interval );
          m_submaps_by_time.put( pulled_interval, sbmp_target );
        }

        pose_t T_m1_m2 = sbmp_target->mapper_pose().inverse() * sbmp_source->mapper_pose();
        sbmp_source->set_pose( sbmp_target->pose() * T_m1_m2 );
        sbmp_target->copy_into( *sbmp_source );
        sbmp_target->clear_by_erode( 2 );

        render_submap( sbmp_target, true );
      }
    }
  }

  std::vector<coord_t> grid_centers;  
  for ( auto & [_, cell] : m_global_voxel.cells() )
  {
    grid_centers.push_back( coord_from_vec( t_from_T( cell->pose() ) ) );
  }

  for ( auto& [_, vis] : m_visualizers )
  {
    vis->reinsert();
  }

  m_grid_pub->publish( pcd_from_coord( grid_centers, "mapper_frame" ) );
  m_pbuf->publish_pose_cloud();
  m_occ_pub->publish( m_pcd.msg() );
  m_sem_pub->publish( m_sem_pcd->msg() );
}


pose_t GlobalMapper::_extrapolate_pose_at( size_t timestamp_ns )
{
  auto [last_time, T_w_m_latest] = m_pbuf->latest_at( timestamp_ns );
  pose_t T_w_t_latest = m_ptbuf->pose_at( last_time );
  pose_t T_w_t_now = m_ptbuf->pose_at( timestamp_ns );
  pose_t T_latest_now = T_w_t_latest.inverse() * T_w_t_now;
  return T_w_m_latest * T_latest_now;
}

void GlobalMapper::timer_cb_single()
{
  std::tuple<sbmp_t, pmap_t, smap_t> element;
  {
    std::lock_guard<std::mutex> lock_queue ( m_mutex_queue );
    m_background_timer_once->cancel();
    assert( !m_queue.empty() );
    element = m_queue.front();
    m_queue.pop();
  }

  auto& sbmp = std::get<0>(element);
  auto& poses = std::get<1>(element);
  auto& images = std::get<2>(element);

  auto poses_it = poses.begin();
  auto images_it = images.begin();

  while ( poses_it != poses.end() )
  {
    RCLCPP_INFO( get_logger(), "projecting unused image at t %lu", poses_it->first );

    auto& pose = (poses_it++)->second;
    auto& image = (images_it++)->second;

    ProjectionBuffer pbuf ( sbmp->pose().inverse() * pose, m_projection_params );
    pbuf.project( sbmp->tree() );
    sbmp->insert_new_records( pbuf.sem(), pbuf.counts(), pbuf.keys(), pbuf.times() );
  }

  sbmp->set_seq_id(m_count++);
  sbmp->set_mapper_pose( sbmp->pose() );
  sbmp->compute_unique_keys( m_global_voxel.tree_params() );

  {
    std::lock_guard<std::mutex> lock_map ( m_mutex_maps );    
    m_submaps_all[sbmp->seq_id()] = sbmp;
    m_submaps_by_time.put( sbmp->intervals().front(), sbmp );
    m_last_map_end.store( sbmp->intervals().front().stop() );
  }

  render_submap( sbmp, true );
  RCLCPP_INFO( get_logger(), "putting interval %lu -> [%lu, %lu]", sbmp->stamp(), sbmp->intervals().front().start(), sbmp->intervals().front().stop() );

  m_occ_pub->publish( m_pcd.msg() );

}

void GlobalMapper::render_submap( sbmp_t sbmp, bool reinsert )
{
  if ( reinsert )
  {
    auto pts = sbmp->tree().filtered_pts();
    if ( !m_pcd.has_idx( sbmp->seq_id() ) )
    {
      m_pcd.put( sbmp->seq_id(), sbmp->tree().filtered_pts(), sbmp->mapper_pose() );
    }
    else
    {
      m_pcd.reinsert( sbmp->seq_id(), pts, sbmp->mapper_pose() );
    }
  }
  else
  {
    m_pcd.update_pose( sbmp->seq_id(), sbmp->mapper_pose() );
  }

  pose_t T_sem_last = m_submap_last_render_sem[sbmp->seq_id()];
  if ( reinsert || delta_t( sbmp->mapper_pose(), T_sem_last ) > 1.0  )
  {

    sbmp->compute_unique_keys( m_global_voxel.tree_params() );
    auto map_cells = m_global_voxel.at_set( sbmp->grid_keys() );
    auto records = sbmp->all_records();

    for ( auto& vm : map_cells )
    {
      size_t cell_index = vm->params().count;

      RCLCPP_INFO( get_logger(), "clearing  sbmp %lu from vm %lu", sbmp->seq_id(), cell_index );
      vm->erase( sbmp->seq_id(), T_sem_last, records );

      RCLCPP_INFO( get_logger(), "inserting sbmp %lu into vm %lu", sbmp->seq_id(), cell_index );
      vm->put( sbmp->seq_id(), sbmp->mapper_pose(), records );

      if ( m_visualizers.find( vm->params().count ) == m_visualizers.end() )
      {
        m_visualizers[cell_index] = std::make_shared<VoxelMapVisualizer>( vm, m_sem_pcd );
      }
      else
      {
        m_visualizers[cell_index]->mark_should_reinsert();
      }
    }

    m_submap_last_render_sem[sbmp->seq_id()] = sbmp->mapper_pose();
  }

}


std::vector<TimeInterval> GlobalMapper::remove_submap( sbmp_t sbmp )
{
  std::vector<TimeInterval> pulled_intervals;

  RCLCPP_INFO( get_logger(), "erasing submap %lu from sbmp by time", sbmp->seq_id() );
  for ( auto& interval : sbmp->intervals() )
  {
    RCLCPP_INFO( get_logger(), "pulling interval [%lu, %lu]", interval.start(), interval.stop() );
    m_submaps_by_time.pull( interval.start() );
    pulled_intervals.push_back( interval );
  }

  auto map_cells = m_global_voxel.at_set( sbmp->grid_keys() );
  for ( auto &vm : map_cells )
  {
    RCLCPP_INFO( get_logger(), "erasing submap %lu from block %lu", sbmp->seq_id(), vm->params().count );
    vm->erase( sbmp->seq_id() );

    m_visualizers[vm->params().count]->mark_should_reinsert();
  }

  m_pcd.remove( sbmp->seq_id() );
  m_submaps_all.erase( sbmp->seq_id() );

  return pulled_intervals;
}