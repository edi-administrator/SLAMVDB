#pragma once

#include "utils.hxx"
#include <functional>
#include <string>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include <optional>
#include <list>
#include "math_utils.hxx"


namespace mvdb
{

class Octree;
struct OctreeParams;

ockey_t coord_to_ockey( size_t depth, double res, const coord_t& coord, size_t maxdepth );

coord_t ockey_to_coord( size_t depth, double res, const ockey_t& key, size_t maxdepth );

size_t ockey_to_index( size_t depth, const ockey_t& key, size_t maxdepth );

size_t last_same_parent_depth( const ockey_t& a, const ockey_t& b, size_t max_depth );

xkey_t ockey_to_xkey( const ockey_t& key, size_t max_depth );

ockey_t xkey_to_ockey( xkey_t xkey, size_t max_depth );

std::string str(const coord_t&);
std::string str(const ockey_t&);
std::vector<ockey_t> key_rays( const coord_t& start, const coord_t& end, double res, size_t depth, size_t max_depth, double clear_dist );
vec_t t_initial( const vec_t& coord, const vec_t& direction, double resolution );
double prob_from_log( double log );
double log_from_prob( double prob );


pose_t grid_pose( const vec_t& t, const OctreeParams& params );
pose_t grid_pose( const pose_t& T, const OctreeParams& params );

class OctreeNode 
{
  public:

    using ptr_t = std::shared_ptr<OctreeNode>;
    OctreeNode( const size_t& depth, const OctreeParams& params, const ockey_t& key = { } );
    virtual ~OctreeNode() {};

    virtual void traverse_recurse( std::function<void(OctreeNode&)> callback, std::function<bool(OctreeNode&)> predicate );
    virtual std::string repr() const;

    static std::function<bool(const OctreeNode&)> make_predicate( const SpatialConstraint& filter );

    inline const coord_t& coord() const { return m_coord; }
    inline const ockey_t& key() const { return m_key; }
    inline const std::array<vec_t, 8>& bbox() const { return m_bbox; }
    virtual bool should_clear() const = 0;
    virtual bool is_leaf() const = 0;
    virtual void clear() = 0;
    friend std::ostream& operator<<(std::ostream&, const OctreeNode&);
  
  protected:
    std::array<vec_t, 8> _bbox();
    std::array<size_t,3> m_key;
    std::array<vec_t, 8> m_bbox;
    size_t m_depth;
    coord_t m_coord;
    const OctreeParams& m_params;
};


class OctreeInternalNode : public OctreeNode
{
  public:
    using ptr_t = OctreeNode::ptr_t;

    template<typename ... Args>
    OctreeInternalNode( Args&& ... args ) : OctreeNode( std::forward<Args>(args) ... )
    {
      m_children = {};
    }

    ~OctreeInternalNode() = default;

    bool is_leaf() const override { return false; }
    bool should_clear() const override;
    void clear() override;
    void traverse_recurse( std::function<void(OctreeNode&)> callback, std::function<bool(OctreeNode&)> predicate ) override;

    friend class Octree;

  protected:
    std::array<std::shared_ptr<OctreeNode>,8> m_children;
};

class OctreeLeafOccNode : public OctreeNode
{
  public:
    using ptr_t = OctreeNode::ptr_t;

    template<typename... Args>
    OctreeLeafOccNode( size_t n_update, Args&& ... args ) : m_occ(0),  m_n_last_update(n_update), OctreeNode( std::forward<Args>(args) ...) {};

    ~OctreeLeafOccNode() = default;

    double occ() const { return m_occ; }
    double prob() const { return prob_from_log(m_occ); }
    bool is_leaf() const override { return true; }
    void clear() override { m_occ = std::numeric_limits<double>::lowest(); };
    bool should_clear() const override;
    void update( bool hit, size_t n_update );
    void reset( double occ, bool add = false );

    size_t m_listpos;

  protected:
    double m_occ;
    size_t m_n_last_update;
    const double log_odds_hit = log_from_prob(0.7);
    const double log_odds_miss = log_from_prob(0.4);
};

struct OctreeParams
{
  coord_t origin = coord_t { 0,0,0 };
  size_t maxdepth = 16;
  double res = 0.25;
  size_t downsample = 0;
  double clear_dist = 25;

  inline double max_extent() const
  {
    return pow(2, maxdepth - 1) * res;
  }

  inline ockey_t coord_to_ockey( size_t depth, const coord_t& coord ) const
  { 
    return mvdb::coord_to_ockey( depth, res, coord, maxdepth );
  }

  inline ockey_t coord_to_ockey_leaf( const coord_t& coord ) const
  { 
    return mvdb::coord_to_ockey( maxdepth - 1, res, coord, maxdepth );
  }

  inline coord_t ockey_to_coord( size_t depth, const ockey_t& key ) const
  {
    return mvdb::ockey_to_coord( depth, res, key, maxdepth );
  }

  inline coord_t ockey_to_coord_leaf( const ockey_t& key ) const
  {
    return mvdb::ockey_to_coord( maxdepth - 1, res, key, maxdepth );
  }

  inline size_t ockey_to_index( size_t depth, const ockey_t& key ) const
  {
    return mvdb::ockey_to_index( depth, key, maxdepth );
  }

  inline size_t ockey_to_index_leaf( const ockey_t& key ) const
  {
    return mvdb::ockey_to_index( maxdepth - 1, key, maxdepth );
  }

  inline size_t last_same_parent_depth( const ockey_t& a, const ockey_t& b ) const
  {
    return mvdb::last_same_parent_depth( a, b, maxdepth );
  }

  inline xkey_t ockey_to_xkey( const ockey_t& key ) const
  {
    return mvdb::ockey_to_xkey( key, maxdepth );
  }

  inline xkey_t translation_to_xkey_leaf( const pose_t& T ) const
  {
    auto key = coord_to_ockey_leaf( coord_from_vec( t_from_T( T ) ) );
    return ockey_to_xkey( key );
  }

  inline ockey_t xkey_to_ockey( u_int64_t ikey ) const
  {
    return mvdb::xkey_to_ockey( ikey, maxdepth );
  }

  inline coord_t xkey_to_coord_leaf( xkey_t xkey ) const
  {
    return ockey_to_coord_leaf( xkey_to_ockey( xkey ) );
  }

  inline vec_t xkey_to_vec_leaf( xkey_t xkey ) const
  {
    return vec_from_coord( xkey_to_coord_leaf( xkey ) );
  }

  inline xkey_t coord_to_xkey_leaf( const coord_t& coord ) const
  {
    return ockey_to_xkey( coord_to_ockey_leaf( coord ) );
  }

  inline xkey_t vec_to_xkey_leaf( const vec_t& vec ) const
  {
    return coord_to_xkey_leaf( coord_from_vec( vec ) );
  }

  inline xkey_t vec_to_xkey_leaf_centered( const vec_t& vec ) const
  {
    return vec_to_xkey_leaf( vec.array() + res / 2 );
  }

  inline vec_t xkey_to_vec_leaf_centered( xkey_t xkey ) const
  {
    return xkey_to_vec_leaf( xkey ).array() - res / 2;
  }

  inline vec_t::Scalar xkey_to_z_leaf( xkey_t xkey ) const
  {
    return xkey_to_vec_leaf( xkey )[2];
  }

  inline xkey_t xkey_to_xy0_xkey_leaf( xkey_t xkey ) const
  {
    vec_t xyz = xkey_to_vec_leaf( xkey );
    return vec_to_xkey_leaf( vec_t { xyz[0], xyz[1], 0 } );
  }

};


class Octree
{
  public:

    Octree( const OctreeParams& params = OctreeParams {} );

    static Octree make_copy( const Octree& other,
       const pose_t& T_other_new = pose_t::Identity(), 
       const SpatialConstraint& filter = IdentityConstraint() );

    void copy_from( const Octree& other,
       const pose_t& T_other_new = pose_t::Identity(),
       const SpatialConstraint& filter = IdentityConstraint(),
       bool should_add = false );

    std::optional<std::shared_ptr<OctreeLeafOccNode>> insert( const coord_t& xyz );
    std::vector<ockey_t> fill_keys( const std::vector<coord_t>& points );
    std::vector<ockey_t> miss_keys( const coord_t& ray_origin, const std::vector<coord_t>& points );

    void find_traverse( 
      std::vector<ockey_t> sorted_keys, 
      std::function<void(OctreeNode&)> callback, 
      size_t terminal_depth, 
      std::function<bool(OctreeNode&)> predicate = [](OctreeNode&){ return true; } 
    ) const;
    
    void insert_scan( 
      const std::vector<coord_t>& points, const coord_t& sensor_origin, 
       bool occlusion = false, std::function<void(const OctreeNode&)> collect_callback = []( const OctreeNode& ){} );

    std::list<xkey_t> erode( size_t count = 1 );

    std::list<xkey_t> clear_by_scan_key_codes( const std::vector<coord_t>& points, const coord_t& sensor_origin );

    xkey_t vec_to_xkey( const vec_t& vec ) const;
    xkey_t coord_to_xkey( const coord_t& vec ) const;
    vec_t xkey_to_vec( const xkey_t& xkey ) const;
    coord_t xkey_to_coord( const xkey_t& xkey ) const;


    std::vector<coord_t> filtered_pts( const SpatialConstraint& constr = IdentityConstraint() ) const;
    std::vector<xkey_t> filtered_keys( const SpatialConstraint& constr = IdentityConstraint() ) const;
    static coord_t transform_pt( const coord_t& coord, const pose_t& T );
    static std::vector<coord_t> transform_pts( const std::vector<coord_t>& coords, const pose_t& T );


    inline void traverse( 
      std::function<void(OctreeNode&)> callback, std::function<bool(OctreeNode&)> predicate = [](OctreeNode&){ return true; }
     ) 
    { 
      m_root->traverse_recurse( callback, predicate ); 
    }

    inline void ctraverse( std::function<void(const OctreeNode&)> callback, std::function<bool(const OctreeNode&)> predicate ) const
    {
      m_root->traverse_recurse( callback, predicate );
    }

    inline void traverse( std::function<void(OctreeNode&)> callback, const SpatialConstraint& filter )
    {
      traverse( callback, OctreeNode::make_predicate( filter ) );
    }

    inline void ctraverse( std::function<void(const OctreeNode&)> callback, const SpatialConstraint& filter ) const
    {
      ctraverse( callback, OctreeNode::make_predicate( filter ) );
    }


    inline void new_update() { m_n_updates++; }
    inline const double& resolution() const { return m_res; }
    inline const double& extent() const { return m_maxextent; }
    inline const size_t& depth() const { return m_maxdepth; }
    inline const coord_t& origin() const { return m_origin; }
    inline const size_t& updates() const { return m_n_updates; }
    inline const size_t& downsample() const { return m_raycast_downsample; }
    inline const OctreeParams& params() const { return m_params; }

    void incr() { m_node_counter++; }
    void decr() { m_node_counter--; }

    friend std::ostream& operator<<( std::ostream& o, const Octree& tree );
  
  protected:
    double m_res;
    double m_maxextent;
    size_t m_maxdepth;
    size_t m_n_updates;
    size_t m_raycast_downsample;
    size_t m_node_counter;
    std::array<double,3> m_origin;
    std::shared_ptr<OctreeNode> m_root;
    const OctreeParams m_params;

};

}