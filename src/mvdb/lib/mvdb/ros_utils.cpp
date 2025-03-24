#include "ros_utils.hxx"

namespace mvdb
{

size_t stamp_to_ns( const builtin_interfaces::msg::Time& stamp )
{
  return size_t(stamp.sec) * size_t(1e9) + size_t(stamp.nanosec);
}
size_t header_to_ns( const std_msgs::msg::Header& h )
{
  return stamp_to_ns(h.stamp);
}

builtin_interfaces::msg::Time ns_to_stamp( size_t timestamp_ns )
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = timestamp_ns / size_t(1e9);
  stamp.nanosec = timestamp_ns % size_t(1e9);
  return stamp;
}

std_msgs::msg::Header ns_to_header( size_t timestamp_ns )
{
  std_msgs::msg::Header h;
  h.stamp = ns_to_stamp(timestamp_ns);
  return h;
}

std::vector<coord_t> coord_from_pcd( const sensor_msgs::msg::PointCloud2& pcd, const SpatialConstraint& filter )
{  
  auto find_c = [&pcd]( const char* c ) -> const sensor_msgs::msg::PointField&
  {
    auto it = std::find_if( 
      pcd.fields.begin(), 
      pcd.fields.end(), 
      [c]( const sensor_msgs::msg::PointField& v ) -> bool
      {
        return v.name == c;
      }
    );
    return *it;
  };

  auto field_x = find_c("x");
  auto field_y = find_c("y");
  auto field_z = find_c("z");

  auto point_step = pcd.point_step;
  auto total = pcd.height * pcd.width;

  assert( !pcd.is_bigendian );                                          // who cares
  assert( field_x.datatype == sensor_msgs::msg::PointField::FLOAT32 );  // who cares
  assert( field_y.datatype == sensor_msgs::msg::PointField::FLOAT32 );  // who cares
  assert( field_z.datatype == sensor_msgs::msg::PointField::FLOAT32 );  // who cares
  
  std::vector<coord_t> coords;
  coords.reserve( total );

  for ( auto dptr = pcd.data.data(); dptr < pcd.data.data() + total * point_step; dptr += point_step )
  {
    float x = *reinterpret_cast<const float*>(dptr + field_x.offset);
    float y = *reinterpret_cast<const float*>(dptr + field_y.offset);
    float z = *reinterpret_cast<const float*>(dptr + field_z.offset);
    coord_t c { x, y, z };
    if ( filter.within(c) ) 
    {
      coords.push_back( c );
    }
  }

  return coords;
}


sensor_msgs::msg::PointCloud2 pcd_from_coord( const std::vector<coord_t>& pts, const std::string& parent, bool set_occupancy, const std::vector<double> occ )
{

  sensor_msgs::msg::PointCloud2 msg;
  
  std::vector<std::string> field_names { "x", "y", "z" };
  if ( set_occupancy )
  {
    field_names.push_back( "occupancy" );
  }

  size_t offset = 0;
  for ( auto& c : field_names )
  {
    sensor_msgs::msg::PointField f;
    f.name = c;
    f.offset = 4 * offset++;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }

  // this is not portable at all.

  msg.header.frame_id = parent;
  msg.height = 1;
  msg.width = pts.size();
  msg.point_step = sizeof(float) * field_names.size();
  msg.data.reserve( pts.size() * msg.point_step );
  msg.is_bigendian = false;

  for ( auto i = 0; i < pts.size(); i++ )
  {
    std::array<float, 4> line { float(pts[i][0]), float(pts[i][1]), float(pts[i][2]), set_occupancy ? float(occ[i]) : 0 };
    for ( auto j = 0; j < field_names.size(); j++ )
    {
      auto& coord = line[j];
      float f = _Float32(coord);
      auto bytes = reinterpret_cast<const uint8_t*>(&f);
      for ( auto i = 0; i < sizeof(float); i++ )
      {
        msg.data.push_back(bytes[i]);
      }
    }
  }

  return msg;
}

pose_t pose_from_msg( const geometry_msgs::msg::PoseStamped& msg )
{
  pose_t T = pose_t::Identity();
  T.block<3,1>(0,3) = vec_t { msg.pose.position.x, msg.pose.position.y, msg.pose.position.z };
  
  Eigen::Quaternion<double> q { 
    msg.pose.orientation.w,
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
  };
  T.block<3,3>(0,0) = q.toRotationMatrix();

  return T;
}

pose_t pose_from_transform_msg( const geometry_msgs::msg::TransformStamped& msg )
{
  pose_t T = pose_t::Identity();
  T.block<3,1>(0,3) = vec_t { msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z };
  
  Eigen::Quaternion<double> q { 
    msg.transform.rotation.w,
    msg.transform.rotation.x,
    msg.transform.rotation.y,
    msg.transform.rotation.z,
  };
  T.block<3,3>(0,0) = q.toRotationMatrix();

  return T;
}

geometry_msgs::msg::PoseStamped msg_from_pose( const pose_t& pose, size_t stamp )
{
  Eigen::AngleAxis<double> R;
  R.fromRotationMatrix( pose.block<3,3>(0,0) );

  Eigen::Quaternion<double> q ( R );
  
  geometry_msgs::msg::PoseStamped msg;

  msg.header = ns_to_header( stamp );

  msg.pose.position.x = pose(0,3);
  msg.pose.position.y = pose(1,3);
  msg.pose.position.z = pose(2,3);

  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();

  return msg;
}

geometry_msgs::msg::TransformStamped transform_msg_from_pose( const pose_t& pose, size_t stamp )
{
  Eigen::AngleAxis<double> R;
  R.fromRotationMatrix( pose.block<3,3>(0,0) );

  Eigen::Quaternion<double> q ( R );
  
  geometry_msgs::msg::TransformStamped msg;

  msg.header = ns_to_header( stamp );

  msg.transform.translation.x = pose(0,3);
  msg.transform.translation.y = pose(1,3);
  msg.transform.translation.z = pose(2,3);

  msg.transform.rotation.w = q.w();
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  
  return msg;
}

void splice_pointcloud_msgs( sensor_msgs::msg::PointCloud2& inout, const sensor_msgs::msg::PointCloud2& other )
{
  assert( inout.header.frame_id == other.header.frame_id );
  assert( inout.fields.size() == other.fields.size() );
  for ( size_t i = 0; i < inout.fields.size(); i++ )
  {
    assert( inout.fields[i].count == other.fields[i].count );
    assert( inout.fields[i].name == other.fields[i].name );
    assert( inout.fields[i].datatype == other.fields[i].datatype );
  }
  
  inout.data.insert( inout.data.end(), other.data.begin(), other.data.end() );
  inout.width = inout.data.size() / inout.point_step;
}

MutablePointCloud2::MutablePointCloud2( bool has_color, const std::string& parent )
: m_has_color(has_color), m_count(0UL)
{
  m_msg = std::make_unique<sensor_msgs::msg::PointCloud2>( pcd_from_coord( {}, parent, has_color, {} ) );
}

size_t MutablePointCloud2::put( size_t count, const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& colors )
{
  size_t end_old = m_points.size();  
  m_points.insert( m_points.end(), points.begin(), points.end() );
  if ( m_has_color )
  {
    m_colors.insert( m_colors.end(), colors.begin(), colors.end() );
  }
  size_t end_new = m_points.size();

  auto new_msg = pcd_from_coord( apply_T( pose, points ), "", m_has_color, colors );
  size_t end_old_bytes = m_msg->data.size();
  m_msg->data.insert( m_msg->data.end(), new_msg.data.begin(), new_msg.data.end() );
  size_t end_new_bytes = m_msg->data.size();

  m_msg->width = end_new_bytes / m_msg->point_step;

  m_index_map_points[count] = { end_old, end_new };
  m_index_map_bytes[count] = { end_old_bytes, end_new_bytes };
  m_transforms_current[count] = pose;
  m_transforms_last[count] = pose;

  return count; 
}

size_t MutablePointCloud2::put_incr( const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& color )
{
  return put( m_count++, points, pose, color );
}

void MutablePointCloud2::update_pose( size_t index, const pose_t& pose )
{

  m_transforms_current[index] = pose;
  if ( delta_t( m_transforms_last[index], pose ) > 0.25 )
  {
    m_transforms_last[index] = pose;

    auto& [start, stop] = m_index_map_points[index];
    auto& [start_byte, stop_byte] = m_index_map_bytes[index];
    std::vector<coord_t> pts { m_points.begin() + start, m_points.begin() + stop };
    std::vector<double> colors {};
    if ( m_has_color )
    {
      colors.insert( colors.end(), m_colors.begin() + start, m_colors.begin() + stop );
    }

    auto new_msg = pcd_from_coord( apply_T( pose, pts ), "tracker_frame", m_has_color, colors );
    if ( new_msg.data.size() != stop_byte - start_byte )
    {
      throw std::out_of_range("Message sizes don't line up! new_msg.size(): " + std::to_string(new_msg.data.size()) + "; byte range: " + std::to_string(stop_byte - start_byte) );
    }
    std::copy( new_msg.data.begin(), new_msg.data.end(), m_msg->data.begin() + start_byte );
  }
}

void MutablePointCloud2::remove( size_t index )
{
  auto& [start, stop] = m_index_map_points[index];
  auto& [start_byte, stop_byte] = m_index_map_bytes[index];

  int delta_points = stop - start;
  int delta_bytes = stop_byte - start_byte;

  m_points.erase( m_points.begin() + start, m_points.begin() + stop );
  if ( m_has_color )
  {
    m_colors.erase( m_colors.begin() + start, m_colors.begin() + stop );
  }
  m_msg->data.erase( m_msg->data.begin() + start_byte, m_msg->data.begin() + stop_byte );

  m_msg->width = m_msg->data.size() / m_msg->point_step;

  for ( auto& [i,_] : m_index_map_points )
  {
    if ( i > index )
    {
      auto& [old_start, old_stop] = m_index_map_points[i];
      auto& [old_start_byte, old_stop_byte] = m_index_map_bytes[i];
  
      m_index_map_points[i] = { old_start - delta_points, old_stop - delta_points };
      m_index_map_bytes[i] = { old_start_byte - delta_bytes, old_stop_byte - delta_bytes };

    }
  }

  m_index_map_bytes.erase(index);
  m_index_map_points.erase(index);
  m_transforms_current.erase(index);
  m_transforms_last.erase(index);
}

void MutablePointCloud2::reinsert( size_t index, const std::vector<coord_t>& points, const pose_t& pose, const std::vector<double>& colors )
{
  auto& [start, stop] = m_index_map_points[index];
  auto& [start_byte, stop_byte] = m_index_map_bytes[index];

  auto msg = pcd_from_coord( apply_T( pose, points ), "", m_has_color, colors );

  int delta_points = points.size() - ( stop - start );
  int delta_bytes = msg.data.size() - ( stop_byte - start_byte );

  m_points = splice_vector( m_points, points, start, stop );
  m_msg->data = splice_vector( m_msg->data, msg.data, start_byte, stop_byte );
  if ( m_has_color )
  {
    m_colors = splice_vector( m_colors, colors, start, stop ); 
  }

  m_msg->width = m_msg->data.size() / m_msg->point_step;

  m_index_map_points[index] = { start, stop + delta_points };
  m_index_map_bytes[index] = { start_byte, stop_byte + delta_bytes };
  m_transforms_current[index] = pose;
  m_transforms_last[index] = pose;

  for ( auto& [i,_] : m_index_map_points )
  {
    if ( i > index )
    {
      auto& [old_start, old_stop] = m_index_map_points[i];
      auto& [old_start_byte, old_stop_byte] = m_index_map_bytes[i];
  
      m_index_map_points[i] = { old_start + delta_points, old_stop + delta_points };
      m_index_map_bytes[i] = { old_start_byte + delta_bytes, old_stop_byte + delta_bytes };
    }
  }

}

VoxelMapVisualizer::VoxelMapVisualizer( std::shared_ptr<VoxelLookup> lookup, std::shared_ptr<MutablePointCloud2> pcd )
 : m_pcd( pcd ),  m_map( lookup ), m_should_reinsert( false )
{
  if ( !m_map->params().quantizer )
  {
    throw std::out_of_range("no quantizer provided!");
  }
  update_points();
  std::cerr << "VoxelMapVisualizer" << m_map->params().count << ": (creation) inserting " << m_spoints.size() << " points\n";
  m_pcd->put( lookup->params().count, m_spoints, lookup->pose(), m_scolors );
}

void VoxelMapVisualizer::remove()
{
  if ( m_pcd )
  {
    std::cerr << "VoxelMapVisualizer" << m_map->params().count << ": (removal) removing " << m_spoints.size() << " points\n";
    m_pcd->remove( m_map->params().count );
  }
}

void VoxelMapVisualizer::reinsert()
{
  if ( m_should_reinsert )
  {
    m_should_reinsert = false;
    update_points();
    std::cerr << "VoxelMapVisualizer" << m_map->params().count << ": (update) re-inserting " << m_spoints.size() << " points\n";
    m_pcd->reinsert( m_map->params().count, m_spoints, m_map->pose(), m_scolors );
  }
}

void VoxelMapVisualizer::update_points()
{
  m_spoints.clear();
  m_scolors.clear();

  for ( auto& record : m_map->get_all() )
  {
    m_scolors.push_back( m_map->params().quantizer->color_mono( record.similarities ) );
    m_spoints.push_back( m_map->tree_params().xkey_to_coord_leaf( record.xkey ) );
  }
}

}