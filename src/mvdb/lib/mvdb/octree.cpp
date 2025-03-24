#include "octree.hxx"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <bitset>
#include <stack>
#include <omp.h>
#include <chrono>

namespace mvdb
{

double log_from_prob( double prob )
{
  return log( prob / ( 1 - prob) );
}

double prob_from_log( double log )
{
  return 1 - 1 / ( exp( log ) + 1 );
}


coord_t operator+(coord_t a, coord_t b)
{
  return coord_t { a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}


vec_t _coord_to_vec( const coord_t& coord )
{
  return { coord[0], coord[1], coord[2] };
}

coord_t _vec_to_coord( const vec_t& vec )
{
  return { vec(0), vec(1), vec(2) };
}

size_t argmax( const vec_t& v )
{
  size_t amax = 0;
  double max = v[amax];
  for ( auto i = 0; i < v.size(); i++ )
  {
    if ( v[i] > max )
    {
      amax = i;
      max = v(i);
    }
  }
  return amax;
}

vec_t t_initial( const vec_t& coord, const vec_t& direction, double resolution )
{
  vec_t integer_coord = coord / resolution;
  vec_t unit_direction = direction / ( direction.norm() + k_epsilon ) ;
  vec_t boundaries = ( unit_direction.array() >= 0 ).select( 
    integer_coord.array().floor() + 1, // for exact integer values: add 1; everywhere else == ceil()
    integer_coord.array().ceil() - 1  // for exact integer values: sub 1; everywhere else == floor()
    );
  return ( k_epsilon + ( boundaries - integer_coord ).array() ).abs() / ( unit_direction.array() ).abs() ;
}

std::vector<ockey_t> key_rays( const coord_t& _start, const coord_t& _end, double res, size_t depth, size_t max_depth, double clear_dist )
{
  vec_t start = _coord_to_vec(_start);
  vec_t end = _coord_to_vec(_end);
  vec_t direction = end - start;
  std::vector<ockey_t> keys;

  double resolution_scale = pow(2, max_depth - depth - 1);
  double adjusted_res = res * resolution_scale;

  vec_t step = direction.array().sign() * resolution_scale;
  vec_t t_delta = t_initial(start, direction, adjusted_res);
  vec_t T_delta = t_initial({0,0,0}, direction, adjusted_res);

  double path_length = std::min( direction.norm() / adjusted_res, clear_dist );
  auto key = coord_to_ockey( depth, res, {0,0,0}, max_depth );

  while ((t_delta + T_delta).array().abs().minCoeff() < path_length && keys.size() < 1e3 )
  {
    keys.push_back( 
      coord_to_ockey( depth, res, _start + ockey_to_coord(depth, res, key, max_depth), max_depth ) 
      );
    auto advance_idx = argmax( -t_delta.array().abs() );
    t_delta(advance_idx) += T_delta(advance_idx);
    key[advance_idx] += step(advance_idx);
  }

  return keys;
}

std::string str( const coord_t& coord ) 
{
  std::stringstream ss;
  ss << "[";
  for ( auto i = 0; i < coord.size(); i++ )
  {
    ss << std::setfill('0') << std::setw(6) << std::fixed << std::setprecision(4) <<  coord[i];
    if ( i < coord.size() - 1 ) ss << " ";
  }
  ss << "]";
  return ss.str();
}

std::string str( const ockey_t& key ) 
{
  std::stringstream ss;
  ss << "<";
  for ( auto i = 0; i < key.size(); i++ )
  {
    ss << std::setfill('0') << std::bitset<24>(key[i]);
    if ( i < key.size() - 1 ) ss << " ";
  }
  ss << ">";
  return ss.str();
}

ockey_t coord_to_ockey( size_t depth, double res, const coord_t& coord, size_t maxdepth )
{
  ockey_t out;
  int shift = int(maxdepth) - int(depth) - 1;
  double centering = pow(2, maxdepth-1);
  
  for ( size_t i = 0; i < out.size(); i++ )
  {
    auto i_coord = int64_t( coord[i] / res + centering );
    out[i] = size_t( ( i_coord >> shift ) << shift );
  }
  return out;
}

coord_t ockey_to_coord( size_t depth, double res, const ockey_t& key, size_t maxdepth )
{
  coord_t out;
  int shift = int(maxdepth) - int(depth) - 1;
  double centering = pow(2, maxdepth-1);
  double cell_centering = res * pow(2, shift-1);
  for ( size_t i = 0; i < out.size(); i++ )
  {
    auto i_coord = double( ( (key[i] >> shift) << shift ) ) - centering;
    out[i] = i_coord * res + cell_centering;
  }
  return out;
}

size_t ockey_to_index( size_t depth, const ockey_t& key, size_t maxdepth )
{
  int shift = int(maxdepth) - int(depth) - 1;
  int check_mask = 1 << shift;

  size_t idx = 0;
  if ( key[0] & check_mask ) idx += 4;
  if ( key[1] & check_mask ) idx += 2;
  if ( key[2] & check_mask ) idx += 1;

  return idx;
}

size_t last_same_parent_depth( const ockey_t& a, const ockey_t& b, size_t max_depth )
{
  auto iidx_a = ockey_to_xkey(a, max_depth);
  auto iidx_b = ockey_to_xkey(b, max_depth);
  auto diff = iidx_a ^ iidx_b;
  auto shift = floor( floor( log2( double(diff) ) ) / 3 );
  return max_depth - shift - 2;
}

xkey_t ockey_to_xkey( const ockey_t& key, size_t max_depth )
{
  std::bitset<64> out {};
  for ( size_t i = 0; i < key.size(); i++ )
  {
    for ( size_t j = 0; j <= max_depth; j++ )
    {
      out[j*key.size() + i] = ( key[i] & ( 1 << j ) ) ? 1 : 0;
    }
  }
  return out.to_ullong();
}

ockey_t xkey_to_ockey( xkey_t ikey, size_t max_depth )
{
  ockey_t key {};
  std::bitset<64> bits (ikey);
  for ( size_t i = 0; i < key.size(); i++ )
  {
    std::bitset<64> out {};
    for ( size_t j = 0; j <= max_depth; j++ )
    {
      out[j] = bits[j*key.size()+i];
    }
    key[i] = out.to_ulong();
  }
  return key;
}

pose_t grid_pose( const vec_t& t, const OctreeParams& params )
{
  double min_step = params.res;
  double fixed_point = 1 / min_step;
  vec_t rounded_translation;

  if ( abs( fixed_point ) < 1.0 )
  {
    rounded_translation = ( ( t * fixed_point ).array().round() / fixed_point ).round();
  }
  else
  {
    rounded_translation = ( t * round(fixed_point) ).array().round() / round(fixed_point);
  }

  rounded_translation.array() += min_step / 2;

  return put_t( pose_t::Identity(), rounded_translation );
}

pose_t grid_pose( const pose_t& T, const OctreeParams& params )
{
  return grid_pose( t_from_T(T), params );
}

OctreeNode::OctreeNode( const size_t& depth, const OctreeParams& params, const ockey_t& key )
 : m_params(params), m_key(key), m_depth(depth)
{
  m_coord = m_params.origin + m_params.ockey_to_coord( depth, key );
  m_bbox = _bbox();
}

void OctreeNode::traverse_recurse( std::function<void(OctreeNode&)> callback, std::function<bool(OctreeNode&)> predicate ) 
{
  if ( predicate(*this) ) callback(*this);
}

std::array<vec_t, 8> OctreeNode::_bbox()
{
  std::array<vec_t, 8> box;
  auto it = box.begin();
  double step = m_params.res * pow(2, m_params.maxdepth - m_depth - 1) / 2;
  for ( double dx = -1; dx < 2; dx += 2 )
  {    
    for ( double dy = -1; dy < 2; dy += 2 )
    {    
      for ( double dz = -1; dz < 2; dz += 2 )
      {
        *it = vec_from_coord(m_coord) + vec_t{ dx, dy, dz } * step;
        it++;
      }
    }
  }
  return box;
}

void OctreeInternalNode::traverse_recurse( std::function<void(OctreeNode&)> callback, std::function<bool(OctreeNode&)> predicate ) 
{
  if ( predicate(*this) )
  {
    for ( ptr_t& ptr : m_children )
    {
      if ( ptr )
      {
        ptr->traverse_recurse( callback, predicate );
      }
    }
    callback(*this);
  }
}

std::function<bool(const OctreeNode&)> OctreeNode::make_predicate( const SpatialConstraint& filter )
{
  return [&]( const OctreeNode& n ) -> bool
  {
    if( n.is_leaf() ) 
    {
      auto c = n.coord();
      return filter.within( c );
    }
    return true;
  };
}

bool OctreeInternalNode::should_clear() const
{
  for ( const ptr_t& ptr : m_children )
  {
    if ( ptr ) return false;
  }
  return true;
}

void OctreeInternalNode::clear()
{
  for ( auto i = 0; i < 8; i++ )
  {
    if ( m_children[i] )
    {
      if ( m_children[i]->should_clear() )
      {
        m_children[i] = nullptr;
      }
    }
  }
}

std::string OctreeNode::repr() const 
{ 
  const static char* marks = "-------------------------------------";
  std::stringstream ss;
  ss <<  std::string(marks, marks+m_depth) << " {" << m_depth << "} is_leaf " << is_leaf() << " xyz " << str(m_coord); 
  return ss.str();
};

void OctreeLeafOccNode::update( bool hit, size_t n_update )
{
  if ( n_update > m_n_last_update ) // only update if the caller has incremented idx, to prevent double updates
  {
    m_occ += ( hit ? log_odds_hit : log_odds_miss );
    m_n_last_update = n_update;
  } 
}

void OctreeLeafOccNode::reset( double occ, bool add )
{
  if ( !add ) m_occ = 0;
  m_occ += occ;
  m_n_last_update = 0;
}

bool OctreeLeafOccNode::should_clear() const
{
  return m_occ < 0;
}

std::ostream& operator<<( std::ostream& s, const OctreeNode& n )
{
  return s << n.repr();
}

std::ostream& operator<<( std::ostream& o, const Octree& tree )
{
  return o << "Octree( depth=" << tree.depth() << "; res=" << tree.resolution() << "; nodes=" << tree.m_node_counter << "; upd=" << tree.m_n_updates << " )";
}


Octree::Octree( const OctreeParams& params )
:  m_params(params), m_res(params.res), m_maxdepth(params.maxdepth), m_origin(params.origin), m_n_updates(0), m_raycast_downsample(params.downsample), m_node_counter(1)
{
  m_maxextent = pow(2, m_maxdepth - 1) * m_res;
  m_root = std::static_pointer_cast<OctreeNode>(
    std::make_shared<OctreeInternalNode>(0, m_params, m_params.coord_to_ockey( 0, m_params.origin ) )
  );
}

Octree Octree::make_copy( const Octree& other, const pose_t& T_other_new, const SpatialConstraint& filter )
{
  Octree out { other.params() };
  out.copy_from( other, T_other_new, filter );
  return out;
}

void Octree::copy_from( const Octree& other, const pose_t& T_other_new, const SpatialConstraint& filter, bool should_add )
{
  pose_t T_new_other = T_other_new.inverse();
  auto reinsert_cb = [&]( const OctreeNode& n )
  {
    if ( n.is_leaf() )
    {
      auto nh_opt = insert( transform_pt( n.coord(), T_new_other ) );
      if ( nh_opt.has_value() )
      {
        nh_opt.value()->reset( static_cast<const OctreeLeafOccNode&>(n).occ(), should_add );
      }
    }
  };  
  other.ctraverse( reinsert_cb, filter );
}

coord_t Octree::xkey_to_coord( const xkey_t& xkey ) const
{
  auto ockey = xkey_to_ockey( xkey, m_maxdepth - 1 );
  return ockey_to_coord( m_maxdepth - 1, m_res, ockey, m_maxdepth );
}

xkey_t Octree::coord_to_xkey( const coord_t& coord ) const
{
  auto key = coord_to_ockey( m_maxdepth - 1, m_res, coord, m_maxdepth );
  return ockey_to_xkey( key, m_maxdepth - 1);
}

vec_t Octree::xkey_to_vec( const xkey_t& xkey ) const
{
  return vec_from_coord( xkey_to_coord(xkey) );
}

xkey_t Octree::vec_to_xkey( const vec_t& vec ) const 
{
  return coord_to_xkey( coord_from_vec(vec) );
}

std::optional<std::shared_ptr<OctreeLeafOccNode>> Octree::insert( const coord_t& xyz )
{
  std::optional<std::shared_ptr<OctreeLeafOccNode>> retval;
  
  if ( std::abs(xyz[0]) > m_maxextent || std::abs(xyz[1]) > m_maxextent || std::abs(xyz[2]) > m_maxextent  )
  {
    return retval;
  }

  auto key = coord_to_ockey( m_maxdepth-1, m_res, xyz, m_maxdepth );
  auto node = m_root;
  size_t cur_depth = 0;

  while ( !node->is_leaf() && cur_depth < m_maxdepth )
  {
    cur_depth++;
    auto& inode = *std::static_pointer_cast<OctreeInternalNode>(node);
    size_t next_idx = ockey_to_index( cur_depth, key, m_maxdepth );

    if ( !inode.m_children[next_idx] )
    {
      incr();
      if ( cur_depth == m_maxdepth - 1 )
      {
        inode.m_children[next_idx] = std::static_pointer_cast<OctreeNode>(
          std::make_shared<OctreeLeafOccNode>( m_n_updates, cur_depth, m_params, key )
         );
      }
      else
      {
        inode.m_children[next_idx] = std::static_pointer_cast<OctreeNode>(
           std::make_shared<OctreeInternalNode>( cur_depth, m_params, key )
           );
      }
    }
    node = inode.m_children[next_idx];
  }

  if ( node->is_leaf() )
  {
    retval = std::static_pointer_cast<OctreeLeafOccNode>(node);
  }

  return retval;
 
}

std::vector<ockey_t> Octree::fill_keys( const std::vector<coord_t>& points )
{
  std::vector<ockey_t> fill_keys;

  for ( auto& p : points )
  {
    auto result = insert(p);
    if ( result.has_value() )
    {
      fill_keys.push_back( result.value()->key() );
    }
  }

  return fill_keys;
}

std::vector<ockey_t> Octree::miss_keys( const coord_t& ray_origin, const std::vector<coord_t>& points )
{
  std::vector<ockey_t> miss_keys;

  #pragma omp parallel
  {
    std::list<ockey_t> _miss_keys_l;
    auto thread_idx = omp_get_thread_num();
    auto n_threads = omp_get_num_threads();

    // if ( thread_idx == 0 ) std::cerr << "n_threads = " << n_threads << "\n";

    for ( auto i = thread_idx; i < points.size(); i += n_threads )
    {
      auto ray = key_rays( ray_origin, points[i], m_res, m_maxdepth - 1 - m_raycast_downsample, m_maxdepth, m_params.clear_dist );
      for ( auto& k : ray )
      {
        _miss_keys_l.push_back( k );
      }
    }

    #pragma omp critical
    {
      miss_keys.insert(miss_keys.begin(), _miss_keys_l.begin(), _miss_keys_l.end());
    }
  }

  return miss_keys;
}

void Octree::insert_scan( const std::vector<coord_t>& points, const coord_t& sensor_origin, bool occlusion, std::function<void(const OctreeNode&)> collector )
{

  auto fill_cb = [&]( OctreeNode& n )
  {
    if ( n.is_leaf() ) static_cast<OctreeLeafOccNode&>(n).update(true, updates());
  };

  auto clear_cb = [&]( OctreeNode& n )
  {
    if ( n.is_leaf() ) static_cast<OctreeLeafOccNode&>(n).update(false, updates());
  };

  auto prune_cb = [&]( OctreeNode& n )
  {
    collector(n);
    if ( !n.is_leaf() ) n.clear();
    if ( n.should_clear() ) decr();
  };

  auto fkeys = fill_keys(points);
  new_update();
  find_traverse(fkeys, fill_cb, m_maxdepth);

  if ( occlusion )
  {
    auto mkeys = miss_keys(sensor_origin, points);
    std::cerr << "MISS KEYS COUNT = " << mkeys.size() << "\n";
    find_traverse( mkeys, clear_cb, m_maxdepth - m_raycast_downsample );
    traverse(prune_cb);  
  }

}

std::list<xkey_t> Octree::erode( size_t count )
{
  std::list<xkey_t> out;

  auto erode_cb = [&, count]( OctreeNode& n )
  {
    if ( n.is_leaf() )
    {
      auto& ln = static_cast<OctreeLeafOccNode&>(n);
      for ( size_t i = 1; i <= count; i++ )
      {
        ln.update( false, updates() + i );
      } 
    }
  };

  auto prune_cb = [&]( OctreeNode& n)
  {
    if ( !n.is_leaf() ) n.clear();
    if ( n.should_clear() ) decr();
    if ( n.is_leaf() && n.should_clear() ) out.push_back( m_params.ockey_to_xkey( n.key() ) );
  };

  traverse(erode_cb);
  traverse(prune_cb);
  m_n_updates += count;

  return out;
}

std::list<xkey_t> Octree::clear_by_scan_key_codes( const std::vector<coord_t>& scan, const coord_t& sensor_origin )
{
  std::list<xkey_t> removed;
  auto collector = [&]( const OctreeNode& n ) -> void
  {
    if ( n.is_leaf() && n.should_clear() ) removed.push_back( coord_to_xkey( n.coord() ) );
  };
  insert_scan( scan, sensor_origin, true, collector );
  return removed;
}

std::vector<coord_t> Octree::filtered_pts( const SpatialConstraint& constr ) const
{
  std::vector<coord_t> out;
  auto collector = [&]( const OctreeNode& n) -> void
  {
    if ( n.is_leaf() ) out.push_back( n.coord() );
  };
  ctraverse( collector, OctreeNode::make_predicate(constr) );
  return out;
}

std::vector<xkey_t> Octree::filtered_keys( const SpatialConstraint& constr ) const
{
  std::vector<xkey_t> fkeys;
  auto fcoords = filtered_pts(constr);
  for ( auto& c : fcoords )
  {
    fkeys.push_back( coord_to_xkey(c) );
  }
  return fkeys;
}

coord_t Octree::transform_pt( const coord_t& pt, const pose_t& T )
{
  auto new_pt = T.block<3,3>(0,0) * vec_from_coord(pt) + T.block<3,1>(0,3);
  return coord_from_vec(new_pt);
}

std::vector<coord_t> Octree::transform_pts( const std::vector<coord_t>& pts, const pose_t& T )
{
  std::vector<coord_t> out;
  for ( auto& pt : pts )
  {
    out.push_back( transform_pt(pt, T) );
  }
  return out;
}

void Octree::find_traverse( 
  std::vector<ockey_t> keys, 
  std::function<void(OctreeNode&)> callback, 
  size_t terminal_depth, 
  std::function<bool(OctreeNode&)> predicate ) const
{
  // unsorted version
  for ( auto& key : keys )
  {
    size_t cur_depth = 0;
    auto node_ptr = m_root;
    while ( cur_depth <= terminal_depth - 2 )
    {      
      cur_depth++;
      auto next_idx = ockey_to_index( cur_depth, key, m_maxdepth );
      auto inode = std::static_pointer_cast<OctreeInternalNode>(node_ptr);

      if ( inode->m_children[next_idx] )
      {
        node_ptr = inode->m_children[next_idx];
        if ( cur_depth == terminal_depth - 1 )
        {
          node_ptr->traverse_recurse(callback, predicate);
        }
      }
      else
      {
        break;
      }
    }
  }

}

} // namespace