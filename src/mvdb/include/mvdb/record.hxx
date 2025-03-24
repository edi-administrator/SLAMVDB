#pragma once

#include <unordered_map>
#include <unordered_set>
#include <list>
#include <memory>
#include <Eigen/Dense>
#include "typedefs.hxx"
#include "math_utils.hxx"
#include "octree.hxx"
#include "quantizer.hxx"

namespace mvdb
{

class Submap;

struct Record
{
  xkey_t xkey = xkey_t {}; // coordinate key in local map frame
  sem_t value = sem_t::Zero(); // semantic vector
  dsc_t similarities = dsc_t::Zero(); // pre-computed discrete class similarities
  skey_t skey = skey_t::Zero(); // semantic index key for deletion
  uint64_t incr = 0; // increment counter
  uint64_t timestamp = 0; // timestamp for exp discounting

  void merge_weighted( const Record& other, std::shared_ptr<Quantizer> q );
  static Record merge_weighted( const Record& a, const Record& b, std::shared_ptr<Quantizer> q );
};

/*! \brief Descriptor for semantic index reinsertion operation */
struct ReinsertionOperation
{
  uid_t updated_uid; // if a record's semantics changed, this is its id
  skey_t pull_skey; // if a record's semantics changed, this is its old semantic key
  skey_t push_skey; // if a record's semantics changed, this is its new semantic key
  uid_t remove_id; // if a record is to be removed, this is the id
  skey_t remove_skey; // if a record is to be removed, this is its semantic key
};

/*! \brief Descriptor for global index record merge operation */
struct MergeOperation
{
  xkey_t xyz; // key ( for flexibility )
  uid_t remaining_id; // id that remains at map[key]
  uid_t merged_id; // id that has to be merged into remaining_id
};

template<typename T_key, typename T_value, typename T_meta>
class AbstractIndexView
{
  public:
    virtual ~AbstractIndexView() {};
    inline bool has_updates() const { return !m_updates.empty(); }
    T_meta pop_update()
    {
      auto out = m_updates.front();
      m_updates.pop_front();
      return out;
    }

    inline std::vector<const T_value*> all() const 
    { 
      std::vector<const T_value*> out;
      for ( auto& [k,v] : m_index )
      {
        out.push_back( &v );
      }
      return out;
    }

  protected:
    std::list<T_meta> m_updates;
    std::unordered_map<T_key, T_value> m_index;
};

class Index : public AbstractIndexView<uid_t, Record, ReinsertionOperation>
{
  public:
    ~Index() = default;

    Index() : m_next("0") {};

    ReinsertionOperation insert( const xkey_t& xkey, const sem_t& v, uint64_t incr, uint64_t time_ns, uid_t idx ); // COMPUTE DISCRETE SMILARITIES LOCALLY ON INSERTION
    ReinsertionOperation insert( const xkey_t& xkey, const sem_t& v, uint64_t incr, uint64_t time_ns ); // COMPUTE DISCRETE SMILARITIES LOCALLY ON INSERTION
    ReinsertionOperation insert( const Record& r );

    ReinsertionOperation remove( const uid_t& idx );
    ReinsertionOperation merge_by_add( const uid_t& to_update, const uid_t& to_delete );

    decltype(m_index)::const_iterator at_it( const uid_t& idx ) const;
    const Record* at( const uid_t& idx ) const; // not optional - only accessed using uids from Spatial or Semantic index
    std::vector<const Record*> at( const std::vector<uid_t>& idxs ) const; // not optional - only accessed using uids from Spatial or Semantic index

    friend class Submap;

  protected:
    uid_t m_next;
    std::shared_ptr<Quantizer> m_discr;
};

class SpatialIndex : public AbstractIndexView<xkey_t, std::vector<uid_t>, MergeOperation>
{
  public:
    SpatialIndex( const Octree& tree ) : m_tree_ref(tree) {};
    ~SpatialIndex() = default;

    void insert( const xkey_t& xyz, uid_t uid ); // insert uid at xyz; internally merge; add merged pair to list of merges
    std::optional<uid_t> remove( const xkey_t& xyz );
    std::vector<xkey_t> all_keys() const; // NOTE THAT MOST POINTS IN A SUBMAP'S OCTREE WILL PROBABLY NOT BE KEYS

    std::optional<uid_t> at( const coord_t& xyz ) const; // optional - may not have record
    std::optional<uid_t> at( const xkey_t& xkey ) const; // optional - may not have record
    std::vector<std::optional<uid_t>> at( const std::vector<xkey_t>& xyzs ) const;
    
    const std::unordered_map<xkey_t, uid_t>& get_map() const { return m_key_to_uid; }
    

  protected:
    const Octree& m_tree_ref;
    std::unordered_map<xkey_t, uid_t> m_key_to_uid;
};

class SemanticIndex // Placeholder - first test with colors (contained in records directly)
{
  public:

    void insert( const skey_t& skey, const uid_t& uid )
    {
      // std::cerr << "noop semantic index INSERT with skey=" << skey.placeholder << " uid=" << uid << "\n";
    } // placeholder
    void remove( const skey_t& skey, const uid_t& uid )
    {
      // std::cerr << "noop semantic index REMOVE with skey=" << skey.placeholder << " uid=" << uid << "\n";
    } // placeholder

    std::optional<uid_t> at ( const skey_t& skey ) const;
    std::vector<std::optional<uid_t>> at( const std::vector<skey_t>& skeys ) const;
};

struct SubmapParams
{
  OctreeParams tree_params = OctreeParams {};
  pose_t T_w_s = pose_t::Identity();
  std::shared_ptr<Quantizer> quantizer = nullptr;
};

class Submap
{
  public:

    Submap( const SubmapParams& params = {} );

    Submap& insert_new_records( 
      const std::vector<sem_t>& values, 
      const std::vector<size_t>& counts, 
      const std::vector<xkey_t>& xkeys, 
      const std::vector<uint64_t>& t 
    );

    std::vector<std::shared_ptr<Submap>> 
    split_off( const VoronoiConstraint& filter, const pose_t& keep, const pose_t& drop );

    std::vector<coord_t>
    filter_coords_xidx( const SpatialConstraint& constraint = IdentityConstraint() ) const;

    std::vector<xkey_t> 
    filter_xkeys( const SpatialConstraint& constraint = IdentityConstraint() ) const;

    xkey_t 
    transform_xkey( const xkey_t& xkey, const pose_t& pose ) const;

    std::vector<xkey_t> 
    transform_xkeys( const std::vector<xkey_t>& xkeys, const pose_t& pose ) const;

    std::vector<const Record*> all_records() const { return m_gidx.all(); }

    std::optional<const Record*> 
    get_at_xkey( const xkey_t& xkey ) const;

    std::optional<const Record*> 
    get_at_skey( const skey_t& skey ) const;

    void compute_bbox();
    void compute_unique_keys( const OctreeParams& sample_params );
    void copy_into( const Submap& other, const SpatialConstraint& filter = IdentityConstraint() );
    void insert_scan( const std::vector<coord_t>& scan, const pose_t& T );
    void fill_records_keys( const std::vector<const Record*>& record_ptrs, const std::vector<xkey_t>& xkey );
    void clear_by_keys( const std::list<xkey_t>& cleared_keys );
    void clear_by_scan( const std::vector<coord_t>& scan, const coord_t& center );
    void clear_by_scan( const std::vector<coord_t>& scan, const pose_t& T );
    void clear_by_erode( size_t count = 1 );
    void add_interval( const TimeInterval& interval );

    inline const Record& get_at_uid( const uid_t& uid ) const { return *( m_gidx.at(uid) ); }
    inline void set_pose( const pose_t& pose ) { m_Tws = pose; }
    inline void set_mapper_pose( const pose_t& correction ) { m_Twm = correction; }
    inline void set_stamp( size_t stamp_ns ) { m_pose_stamp = stamp_ns; }
    inline void set_seq_id( size_t seq_id ) { m_seq_id = seq_id; }
    inline const std::unordered_set<xkey_t>& grid_keys() const { return m_unique_keys; }

    inline Octree& tree() { return m_tree; }
    inline const Octree& tree() const { return m_tree; }

    inline const std::vector<coord_t>& corners() const { return m_record_corners; }
    inline const size_t& stamp() const { return m_pose_stamp; }
    inline const size_t& seq_id() const { return m_seq_id; }
    inline const std::vector<TimeInterval>& intervals() const { return m_covered_intervals; }
    inline const pose_t& pose() const { return m_Tws; }
    inline const pose_t& mapper_pose() const { return m_Twm; } 
    inline const Index& global_idx() const { return m_gidx; }
    inline const SpatialIndex& spatial_idx() const { return m_xidx; }
    inline const SemanticIndex& semantic_idx() const { return m_sidx; }
    inline const std::unordered_map<xkey_t, uid_t>& get_xidx_map() const { return m_xidx.get_map(); };

  
  protected:
    std::vector<coord_t> m_record_corners;
    std::unordered_set<xkey_t> m_unique_keys;
    size_t m_seq_id = -1UL;
    size_t m_pose_stamp = -1UL;
    std::vector<TimeInterval> m_covered_intervals = {};
    pose_t m_Tws;
    pose_t m_Twm = pose_t::Identity();
    Index m_gidx;
    SpatialIndex m_xidx;
    SemanticIndex m_sidx;
    Octree m_tree;
    /// TEMPORARY - UNTIL INHERITANCE
    OctreeParams m_tree_params;
    /// TEMPORARY!

};

struct LocalVoxelLookupParams
{
  pose_t pose = pose_t::Identity();
  double side_length = 64.0;
  xkey_t xkey = -1UL;
  size_t count = -1UL;
  OctreeParams tree_params = { .maxdepth = 16, .res = 1.0 };
  OctreeParams submap_tree_params = { .maxdepth = 16, .res = 0.25 };
  std::shared_ptr<Quantizer> quantizer = nullptr;
};

template<typename M>
inline std::vector<typename M::key_type> all_keys( const M& kvmap )
{
  std::vector<typename M::key_type> out;
  for ( auto& [k,_] : kvmap )
  {
    out.push_back(k);
  }
  return out;
}

class SurfaceKeySet
{
  public:
    SurfaceKeySet( const OctreeParams& params = {} ) : m_tree_params( params ) {};
    SurfaceKeySet( const std::vector<xkey_t>& xkeys, const OctreeParams& params = {} );

    void put( const std::vector<xkey_t>& xkeys );

    std::vector<xkey_t> keys() const;


  protected:
    OctreeParams m_tree_params;
    std::unordered_map<xkey_t, std::pair<vec_t::Scalar, xkey_t>> m_surface_keys;
};

class VoxelLookup
{
  using cell_entry_t = std::unordered_set<const Record*>;
  using cell_t = std::unordered_map<size_t, cell_entry_t>;

  static void _put( cell_t& existing, size_t id, const Record* const& _new );
  Record _get( cell_t& existing );

  public:
    VoxelLookup( const LocalVoxelLookupParams& params = {} );

    std::optional<xkey_t> local_key( const pose_t& mapper_pose, xkey_t xkey );
    std::vector<xkey_t> local_occupied_keys( const pose_t& mapper_pose, const std::vector<const Record*>& records );

    void put( size_t seq_id, const pose_t& T, const Record* const& v );
    void put( size_t seq_id, const pose_t& T, const std::vector<const Record*>& records );

    void erase( size_t seq_id, const std::vector<xkey_t>& xkeys );
    void erase( size_t seq_id );
    void erase( size_t seq_id, const pose_t& T, const std::vector<const Record*>& records );

    std::optional<Record> get( xkey_t xkey );
    std::vector<Record> get_surface();
    std::vector<Record> get_all( std::function<bool(const Record&)> predicate = [](const Record&){ return true; } );

    inline const LocalVoxelLookupParams& params() const { return m_params; }
    inline const OctreeParams& tree_params() const { return m_params.tree_params; }
    inline const pose_t& pose() const { return m_pose; }
    inline const xkey_t& xkey() const { return m_xkey;  }


  protected:
    xkey_t m_xkey;
    pose_t m_pose;
    FrustumConstraint m_bbox;
    LocalVoxelLookupParams m_params;
    std::unordered_map<xkey_t, std::unordered_map<size_t, cell_entry_t>> m_map;
};

struct GlobalVoxelMapParams
{
  OctreeParams tree_params = OctreeParams { .maxdepth = 10, .res = 32.0 };
  LocalVoxelLookupParams lookup_params = LocalVoxelLookupParams {  .side_length = 32.0, .tree_params = OctreeParams { .maxdepth = 16, .res = 0.5 } };
  std::shared_ptr<Quantizer> quantizer = nullptr;
};

class GlobalVoxelMap
{
  public:
    GlobalVoxelMap( const GlobalVoxelMapParams& params = {} );

    std::shared_ptr<VoxelLookup> at( xkey_t xkey );
    std::shared_ptr<VoxelLookup> at( const vec_t& translation );
    std::shared_ptr<VoxelLookup> at( const coord_t& coord );

    std::optional<std::shared_ptr<VoxelLookup>> at_filtered( xkey_t xkey, const SpatialConstraint& filter );
    std::optional<std::shared_ptr<VoxelLookup>> at_filtered( const vec_t& translation, const SpatialConstraint& filter );

    std::vector<std::shared_ptr<VoxelLookup>>  at_filtered( const SpatialConstraint& filter );
    std::vector<std::shared_ptr<VoxelLookup>> at_set( const std::unordered_set<xkey_t>& xkeys );

    inline std::unordered_map<xkey_t, std::shared_ptr<VoxelLookup>> cells() { return m_global_map; }
    inline const OctreeParams& tree_params() const { return m_tree_params; }

  protected:
    std::shared_ptr<Quantizer> m_quantizer;
    size_t m_count = 0UL;
    LocalVoxelLookupParams m_default_params;
    OctreeParams m_tree_params;
    std::unordered_map<xkey_t, std::shared_ptr<VoxelLookup>> m_global_map;
};

}