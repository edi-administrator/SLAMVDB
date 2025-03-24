#include "record.hxx"
#include <exception>

namespace mvdb
{

template<typename T>
std::vector<T> reduce_options( const std::vector<std::optional<T>>& v )
{
  std::vector<T> out;
  for ( auto& option : v )
  {
    if ( option.has_value() ) out.push_back( option.value() );
  }
  return out;
}

sem_t add_weighted( const sem_t& a, const sem_t& b, const size_t& n_a, const size_t& n_b, [[maybe_unused]] size_t stamp_a, [[maybe_unused]] size_t stamp_b )
{
  return ( ( double(n_a) * a + double(n_b) * b ) ).normalized();
  // return ( double(n_a) * a + double(n_b) * b ) / double( n_a + n_b );
}

// void Record::merge_weighted( const Record& other, const Quantizer& q )
void Record::merge_weighted( const Record& other, std::shared_ptr<Quantizer> q )
{
  value = add_weighted( value, other.value, incr, other.incr, timestamp, other.timestamp );
  timestamp = std::max( timestamp, other.timestamp );
  skey = ( q ) ? q->semantic_key( value ) : skey_t::Zero();
  similarities = ( q ) ? q->discrete_sim( value ) : dsc_t::Zero();
  incr += other.incr;
}

Record Record::merge_weighted( const Record& a, const Record& b, std::shared_ptr<Quantizer> q )
{
  Record out {};
  out.merge_weighted(a, q);
  out.merge_weighted(b, q);
  return out;
}

decltype(Index::m_index)::const_iterator Index::at_it( const uid_t& idx ) const
{
  auto it = m_index.find( idx );
  if ( it == m_index.end() )
  {
    throw std::out_of_range("uid " + idx + " not present in global index!");
  }
  return it;
}

ReinsertionOperation Index::insert( const xkey_t& xkey, const sem_t& v, uint64_t incr, uint64_t time_ns, uid_t idx )
{
  ReinsertionOperation out;

  skey_t skey = ( m_discr ) ? m_discr->semantic_key(v) : skey_t::Zero();
  dsc_t dsc = ( m_discr ) ? m_discr->discrete_sim(v) : dsc_t::Zero();

  out.updated_uid = m_next;
  out.push_skey = skey;

  m_index.insert_or_assign( idx, Record { .xkey=xkey, .value=v, .similarities=dsc, .incr=incr, .timestamp=time_ns } );

  return out;
}

ReinsertionOperation Index::insert( const xkey_t& xkey, const sem_t& v, uint64_t incr, uint64_t time_ns )
{
  m_next = std::to_string( std::stoi(m_next) + 1 );
  
  auto operation = insert( xkey, v, incr, time_ns, m_next );
  m_updates.push_back(operation);

  return operation;
}

ReinsertionOperation Index::insert( const Record& r )
{
  return insert( r.xkey, r.value, r.incr, r.timestamp );
}

ReinsertionOperation Index::remove( const uid_t& idx )
{ 
  ReinsertionOperation operation;
  auto record = at(idx);
  operation.remove_id = idx;
  operation.remove_skey = record->skey;
  m_index.erase(idx);
  return operation;
}

ReinsertionOperation Index::merge_by_add( const uid_t& to_update, const uid_t& to_delete )
{
  ReinsertionOperation operation;
  auto inplace = at( to_update );
  auto merged = at( to_delete );

  uint64_t new_stamp = std::max(inplace->timestamp, merged->timestamp);
  size_t new_incr = inplace->incr + merged->incr;
  sem_t new_value = add_weighted( 
    inplace->value, 
    merged->value, 
    inplace->incr, 
    merged->incr, 
    inplace->timestamp, 
    merged->timestamp );
  
  operation.updated_uid = to_update;
  operation.remove_id = to_delete;
  operation.remove_skey = merged->skey;
  operation.pull_skey = inplace->skey;

  remove( to_delete );

  auto result = insert( inplace->xkey, new_value, new_incr, new_stamp, to_update );
  operation.push_skey = result.push_skey;
  m_updates.push_back( operation );

  return operation;
}

const Record* Index::at( const uid_t& idx ) const
{
  return &( at_it(idx)->second );
}

std::vector<const Record*> Index::at( const std::vector<uid_t>& idxs ) const
{
  std::vector<const Record*> out;
  for ( auto& idx : idxs )
  {
    out.push_back( at(idx) );
  }
  return out;
}

void SpatialIndex::insert( const xkey_t& xkey, uid_t uid )
{
  auto remaining_id_opt = at(xkey);
  if ( remaining_id_opt.has_value() )
  {
    m_updates.push_back( MergeOperation { xkey, remaining_id_opt.value(), uid } );
  }
  else
  {
    m_key_to_uid.insert( { xkey, uid } );
  }
}

std::optional<uid_t> SpatialIndex::remove( const xkey_t& xkey )
{
  auto result = at(xkey);
  if ( result.has_value() ) m_key_to_uid.erase(xkey);
  return result;
}

std::vector<xkey_t> SpatialIndex::all_keys() const
{
  std::vector<xkey_t> keys;
  for ( auto& [k, _] : m_key_to_uid )
  {
    keys.push_back(k);
  }
  return keys;
}

std::optional<uid_t> SpatialIndex::at( const coord_t& coord ) const
{
  return at( m_tree_ref.coord_to_xkey( coord ) );
}

std::optional<uid_t> SpatialIndex::at( const xkey_t& xkey ) const
{
  std::optional<uid_t> result;
  auto it = m_key_to_uid.find( xkey );
  if ( it != m_key_to_uid.end() )
  {
    result = it->second;
  }
  return result;
}

std::vector<std::optional<uid_t>> SpatialIndex::at( const std::vector<xkey_t>& xkeys ) const
{
  std::vector<std::optional<uid_t>> out;
  for ( auto& xkey : xkeys )
  {
    out.push_back( at(xkey) );
  }
  return out;
}


Submap::Submap( const SubmapParams& params )
: m_tree_params( params.tree_params ),  m_xidx( m_tree ), m_tree( params.tree_params ), m_Tws( params.T_w_s )
{
  m_gidx.m_discr = params.quantizer;
}

std::optional<const Record*> 
Submap::get_at_xkey( const xkey_t& xkey ) const
{
  std::optional<const Record*> out;
  auto uid = m_xidx.at( xkey );
  if ( uid.has_value() )
  {
    out = m_gidx.at(uid.value());
  }
  return out;
}

void Submap::add_interval( const TimeInterval& interval )
{
  m_covered_intervals.push_back(interval);
}

Submap& Submap::insert_new_records( 
  const std::vector<sem_t>& values, const std::vector<size_t>& counts, const std::vector<xkey_t>& xkeys, const std::vector<uint64_t>& t )
{
  std::vector<coord_t> coords;

  for ( auto i = 0; i < values.size(); i++ )
  {
    uint64_t timestamp_now  = t.at(i);
    auto& xkey = xkeys.at(i);
    auto& n = counts.at(i);
    auto& v = values.at(i);
 
    coords.push_back( m_tree_params.xkey_to_coord_leaf(xkey) );

    auto insertion_result = m_gidx.insert( xkey, v, n, timestamp_now );
    m_xidx.insert( xkey, insertion_result.updated_uid );
  }

  while ( m_xidx.has_updates() )
  {
    auto collision_info = m_xidx.pop_update();
    auto reinsert_info = m_gidx.merge_by_add( collision_info.remaining_id, collision_info.merged_id );
  }

  while ( m_gidx.has_updates() )
  {
    auto merge_info = m_gidx.pop_update();
    m_sidx.remove( merge_info.remove_skey, merge_info.remove_id );
    m_sidx.remove( merge_info.pull_skey, merge_info.updated_uid );
    m_sidx.insert( merge_info.push_skey, merge_info.updated_uid );
  }

  return *this;
}

void Submap::compute_bbox()
{
  auto all_coords = filter_coords_xidx();
  if ( all_coords.size() > 0 )
  {
    m_record_corners = cloud_bbox( apply_T( m_Twm, all_coords ) );
  }
}


void Submap::compute_unique_keys( const OctreeParams& sample_params )
{
  m_unique_keys.clear();
  for ( auto& coord : filter_coords_xidx() )
  {
    vec_t v = apply_T( mapper_pose(), vec_from_coord( coord ) );
    m_unique_keys.insert(  sample_params.vec_to_xkey_leaf_centered( v ) );
  }
}

void Submap::clear_by_keys( const std::list<xkey_t>& cleared_keys )
{
  std::cerr << "SUBMAP: cleared_keys.size() = " << cleared_keys.size() << "\n";
  for ( auto& ck : cleared_keys )
  {
    auto clear_uid = m_xidx.remove(ck);
    if ( clear_uid.has_value() )
    {
      auto removal_result = m_gidx.remove( clear_uid.value() );
      m_sidx.remove( removal_result.remove_skey, removal_result.remove_id );
    }
  }
}

void Submap::clear_by_scan( const std::vector<coord_t>& scan, const coord_t& center )
{
  clear_by_keys( m_tree.clear_by_scan_key_codes( scan, center ) );
}


void Submap::clear_by_erode( size_t count )
{
  std::cerr << "SUBMAP: clearing by ERODE; count = " << count << "\n";
  clear_by_keys( m_tree.erode( count ) );
}

void Submap::insert_scan( const std::vector<coord_t>& scan, const pose_t& T )
{
  auto pts = m_tree.transform_pts( scan, m_Tws.inverse() * T );
  m_tree.insert_scan( pts, coord_from_vec(T.block<3,1>(0,3)), true );
}

void Submap::clear_by_scan( const std::vector<coord_t>& scan, const pose_t& T_w_scan )
{
  auto pts = m_tree.transform_pts( scan, m_Tws.inverse() * T_w_scan );
  pose_t T_sbmp_scan = m_Tws.inverse() * T_w_scan;
  clear_by_scan( pts,  coord_from_vec( t_from_T( T_sbmp_scan ) ) );
}

void Submap::fill_records_keys( const std::vector<const Record*>& record_ptrs, const std::vector<xkey_t>& xkeys )
{
  auto new_xkeys = transform_xkeys( xkeys, m_Tws.inverse() );
  std::vector<sem_t> values;
  std::vector<size_t> counts;
  std::vector<uint64_t> times;
  for ( auto r : record_ptrs )
  {
    values.push_back( r->value );
    counts.push_back( r->incr );
    times.push_back( r->timestamp );
  }
  insert_new_records( values, counts, new_xkeys, times );
}


std::vector<std::shared_ptr<Submap>> Submap::split_off( const VoronoiConstraint& filter_keep, const pose_t& keep, const pose_t& drop )
{
  std::vector<std::shared_ptr<Submap>> out;
  CentersConstraint filter_drop { 15, filter_keep.inverse() };

  std::initializer_list<std::tuple<const SpatialConstraint&, const pose_t&>> pairs {
    { filter_keep, keep },
    { filter_drop, drop }
  };

  for ( auto& [filter, T_w_new] : pairs )
  {
    SubmapParams new_params
    {
      .tree_params = m_tree.params(),
      .T_w_s = T_w_new,
      .quantizer = m_gidx.m_discr
    };

    auto tmp = std::make_shared<Submap>( new_params );

    tmp->copy_into( *this, filter );
    out.push_back( tmp );
  }

  return out;
}

void Submap::copy_into( const Submap& other, const SpatialConstraint& filter )
{
  pose_t T_this_other = pose().inverse() * other.pose();
  m_tree.copy_from( other.tree(), T_this_other.inverse(), filter, false );
  auto fkeys_other = other.filter_xkeys( filter );
  auto records_other = other.m_gidx.at( reduce_options( other.m_xidx.at(fkeys_other) ) );
  auto fkeys_world = transform_xkeys( fkeys_other, other.pose() );
  fill_records_keys( records_other, fkeys_world );
}

std::vector<coord_t> Submap::filter_coords_xidx( const SpatialConstraint& constraint ) const
{
  std::vector<coord_t> out;
  for ( auto& xk : filter_xkeys( constraint ) )
  {
    out.push_back( m_tree_params.xkey_to_coord_leaf( xk ) );
  }
  return out;
}

std::vector<xkey_t> Submap::filter_xkeys( const SpatialConstraint& constraint ) const
{
  std::vector<xkey_t> out;
  for ( auto& xk : m_xidx.all_keys() )
  {
    if ( constraint.within( m_tree_params.xkey_to_vec_leaf(xk) ) )
    {
      out.push_back(xk);
    }
  }
  return out;
}

xkey_t Submap::transform_xkey( const xkey_t& xkey, const pose_t& T ) const
{
  return m_tree_params.vec_to_xkey_leaf( apply_T( T, m_tree_params.xkey_to_vec_leaf(xkey) ) );
}

std::vector<xkey_t> Submap::transform_xkeys( const std::vector<xkey_t>& xkeys, const pose_t& pose ) const
{
  std::vector<xkey_t> out;
  for ( auto& xk : xkeys )
  {
    out.push_back( transform_xkey( xk, pose ) );
  }
  return out;
}


GlobalVoxelMap::GlobalVoxelMap( const GlobalVoxelMapParams& params ) 
: m_quantizer( params.quantizer ), m_tree_params( params.tree_params ), m_default_params( params.lookup_params )
{
}

std::shared_ptr<VoxelLookup> GlobalVoxelMap::at( xkey_t xkey )
{
  auto found = m_global_map.find( xkey );
  if ( found == m_global_map.end() )
  {
    pose_t T = translate_T( pose_t::Identity(),  m_tree_params.xkey_to_vec_leaf_centered( xkey ) );
    std::cerr << "inserting new local voxel map; xkey = " << xkey <<  "\n";
    std::cerr << "inserting new local voxel map; coord= " << t_from_T( T ).transpose() <<  "\n";
    LocalVoxelLookupParams params 
    { 
      .pose = T,
      .side_length = m_default_params.side_length,
      .xkey = xkey,
      .count = m_count++, 
      .tree_params = m_default_params.tree_params,
      .quantizer = m_quantizer,
    };

    m_global_map.insert_or_assign( xkey, std::make_shared<VoxelLookup>( params ) );

    std::cerr << "total count of maps = " << m_global_map.size() <<  "\n";

    for ( auto& [xk, m] : m_global_map )
    {
      std::cerr << "map " << m->params().count << " center = " << m_tree_params.xkey_to_vec_leaf( xk ).transpose() << "\n";
    }
  }
  else
  {
    std::cerr << "already found local voxel map; xkey = " << xkey <<  "\n";
    std::cerr << "total count of maps = " << m_global_map.size() <<  "\n";
  }

  return m_global_map[xkey];
}

std::shared_ptr<VoxelLookup> GlobalVoxelMap::at( const coord_t& coord )
{
  return at( vec_from_coord( coord ) );
}

std::vector<std::shared_ptr<VoxelLookup>> GlobalVoxelMap::at_filtered( const SpatialConstraint& filter )
{
  std::vector<std::shared_ptr<VoxelLookup>> out;
  for ( auto& [xkey,_] : m_global_map )
  {
    auto result_opt = at_filtered( xkey, filter );
    if ( result_opt.has_value() )
    {
      out.push_back( result_opt.value() );
    }
  } 
  return out;
}

std::optional<std::shared_ptr<VoxelLookup>> GlobalVoxelMap::at_filtered( xkey_t xkey, const SpatialConstraint& filter )
{
  return at_filtered( m_tree_params.xkey_to_vec_leaf_centered( xkey ), filter );
}

std::optional<std::shared_ptr<VoxelLookup>> GlobalVoxelMap::at_filtered( const vec_t& translation, const SpatialConstraint& filter )
{
  std::optional<std::shared_ptr<VoxelLookup>> out;
  if ( filter.within( translation ) )
  {
    out = at( translation );
  }
  return out;
}

std::shared_ptr<VoxelLookup> GlobalVoxelMap::at( const vec_t& translation )
{
  auto xkey = m_tree_params.vec_to_xkey_leaf_centered( translation );
  return at( xkey );
}

std::vector<std::shared_ptr<VoxelLookup>> GlobalVoxelMap::at_set( const std::unordered_set<xkey_t>& xkeys )
{

  std::vector<std::shared_ptr<VoxelLookup>> out;

  for ( auto& xkey : xkeys )
  {
    auto vl = at( xkey );
    if ( std::find( out.begin(), out.end(), vl ) == out.end() )
    {
      out.push_back(vl);
    }
  }

  return out;
}


SurfaceKeySet::SurfaceKeySet( const std::vector<xkey_t>& xkeys, const OctreeParams& params )
 : m_tree_params( params )
{
  put( xkeys );
}


void SurfaceKeySet::put( const std::vector<xkey_t>& xkeys )
{
  m_surface_keys.clear();
  for ( auto& xkey : xkeys )
  {
    auto xy0key = m_tree_params.xkey_to_xy0_xkey_leaf( xkey );
    auto z = m_tree_params.xkey_to_z_leaf( xkey );
    auto find = m_surface_keys.find( xy0key );
    // if ( find == m_surface_keys.end() || ( find->first - z < m_tree_params.res * 2 && z < find->first ) )
    if ( find == m_surface_keys.end() || ( z - find->first < m_tree_params.res * 2 && z > find->first ) )
    {
      m_surface_keys[xy0key] = { z, xkey };
    }
  }
}

std::vector<xkey_t> SurfaceKeySet::keys() const
{
  std::vector<xkey_t> out;
  for ( auto& [_,z_xkey] : m_surface_keys )
  {
    out.push_back( z_xkey.second );
  }
  return out;
}

VoxelLookup::VoxelLookup( const LocalVoxelLookupParams& params )
: m_bbox( FrustumConstraint::cube( pose_t::Identity(), params.side_length ) ), m_params( params ), m_pose( params.pose ), m_xkey( params.xkey )
{
  std::cerr << "VoxelLookup: pose = \n";
  std::cerr << params.pose << "\n";
  std::cerr << "VoxelLookup: maxdepth   = " <<  params.tree_params.maxdepth  << "\n";
  std::cerr << "VoxelLookup: max_extent = " <<  params.tree_params.max_extent()  << "\n";
  std::cerr << "VoxelLookup: side_length= " <<  params.side_length  << "\n";
  std::cerr << "VoxelLookup: res  = " << params.tree_params.res << "\n";
};

std::optional<xkey_t> VoxelLookup::local_key( const pose_t& mapper_pose, xkey_t xkey )
{
  std::optional<xkey_t> out;
  auto vec_sframe = m_params.submap_tree_params.xkey_to_vec_leaf( xkey );
  auto vec_lframe = apply_T( m_pose.inverse() * mapper_pose, vec_sframe );
  
  if ( m_bbox.within( vec_lframe ) )
  {
    out = m_params.tree_params.vec_to_xkey_leaf( vec_lframe );
  }
  
  return out;
}

std::vector<xkey_t> VoxelLookup::local_occupied_keys( const pose_t& mapper_pose, const std::vector<const Record*>& records )
{
  std::unordered_set<xkey_t> out;
  for ( auto& record : records )
  {
    auto value_opt = local_key( mapper_pose, record->xkey );
    if ( value_opt.has_value() && m_map.find( value_opt.value() ) != m_map.end() )
    {
      out.insert( value_opt.value() );
    }
  }
  return std::vector<xkey_t> { out.begin(), out.end() };
}

void VoxelLookup::_put( cell_t& existing, size_t id, const Record* const& _new )
{
  existing[id].insert( _new );
}

void VoxelLookup::put( size_t seq_id, const pose_t& T, const Record* const& v )
{
  auto xkey_lframe = local_key( T, v->xkey );
  if ( xkey_lframe.has_value() )
  {
    auto cell = m_map.find( xkey_lframe.value() );
    if ( cell == m_map.end() )
    {
      m_map.insert_or_assign( xkey_lframe.value(), cell_t {} );
      cell = m_map.find( xkey_lframe.value() );
    }
    _put( cell->second, seq_id, v );
  }
}

void VoxelLookup::put( size_t seq_id, const pose_t& T, const std::vector<const Record*>& records )
{
  for ( auto& record : records )
  {
    put( seq_id, T, record );
  }
}

void VoxelLookup::erase( size_t seq_id, const std::vector<xkey_t>& xkeys )
{
  std::vector<xkey_t> to_erase;
  for ( auto& xkey : xkeys )
  {
    auto find = m_map.find(xkey);
    if ( find != m_map.end() )
    {
      auto& cell = find->second;
      if ( cell.find( seq_id ) != cell.end() )
      {
        cell.erase( seq_id );
      }
      if ( cell.empty() )
      {
        to_erase.push_back( xkey );
      }
    }
  }
  std::cerr << "visited (using lookup) " << xkeys.size() << " / " << m_map.size() << " cells in block " << m_params.count << "\n";
  std::cerr << "cleared and erasing    " << to_erase.size() << " / " << m_map.size() << " cells in block " << m_params.count << "\n";
  for ( auto& key : to_erase )
  {
    m_map.erase( key );
  }
}

void VoxelLookup::erase( size_t seq_id, const pose_t& T, const std::vector<const Record*>& records )
{
  auto local_xkeys = local_occupied_keys( T, records );
  erase( seq_id, local_xkeys );
}

void VoxelLookup::erase( size_t seq_id )
{
  std::vector<xkey_t> all_keys;
  for ( auto& [k,_] : m_map )
  {
    all_keys.push_back(k);
  }
  erase( seq_id, all_keys );
}

Record VoxelLookup::_get( cell_t& existing )
{
  Record out {};
  auto outer_it = existing.begin();
  while ( outer_it != existing.end() )
  {
    auto inner_it = outer_it->second.begin();
    while ( inner_it != outer_it->second.end() )
    {
      out.merge_weighted( **inner_it++, m_params.quantizer );
    }
    outer_it++;
  }
  return out;
}

std::optional<Record> VoxelLookup::get( xkey_t xkey )
{
  std::optional<Record> out;
  auto cell = m_map.find( xkey );
  if ( cell != m_map.end() )
  {
    out = _get( cell->second );
  }
  return out;
}

std::vector<Record> VoxelLookup::get_all( std::function<bool(const Record&)> predicate )
{
  std::vector<Record> out;
  for ( auto& [xkey_local, cell] : m_map )
  {
    auto record = _get( cell );
    if ( predicate( record ) )
    {
      record.xkey = xkey_local;
      out.push_back( record );
    }
  }
  
  return out;
}

std::vector<Record> VoxelLookup::get_surface()
{
  std::vector<Record> out;
  SurfaceKeySet surface { all_keys( m_map ), m_params.tree_params };
  for ( auto& surface_key : surface.keys() )
  {
    auto record_opt = get( surface_key );
    if ( record_opt.has_value() )
    {
      auto& record = record_opt.value();
      record.xkey = surface_key;
      out.push_back( record );
    }
  }
  return out;
}

}