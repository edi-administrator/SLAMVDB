#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <numeric>
#include <map>
#include <optional>
#include "typedefs.hxx"

namespace mvdb
{

template<typename T, size_t n>
std::vector<T> vector_from_array( const std::array<T,n>& );

template<typename T, size_t n>
std::array<T, n> array_from_vector( const std::vector<T>& );

vec_t vec_from_coord( const coord_t& coord );
coord_t coord_from_vec( const vec_t& vec );

vec_t::Scalar delta_t( const pose_t& a, const pose_t& b );
vec_t::Scalar path_length( const std::vector<pose_t>& poses );

vec_t apply_T( const pose_t& T, const vec_t& vec );
coord_t apply_T( const pose_t& T, const coord_t& vec );
std::vector<vec_t> apply_T( const pose_t& T, const std::vector<vec_t>& vec );
std::vector<coord_t> apply_T( const pose_t& T, const std::vector<coord_t>& vec );

Eigen::Matrix3d R_from_T( const pose_t& T ); 

pose_t put_t( const pose_t& T, const vec_t& t );
vec_t t_from_T( const pose_t& pose ); 
std::vector<vec_t> t_from_T( const std::vector<pose_t>& poses );
pose_t translate_T( const pose_t& T, const vec_t& delta );

pose_t interpolate_pose( const pose_t& a, const pose_t& b, double t_a, double t_b, double t );

std::vector<coord_t> box_corners_canonical();
std::array<coord_t, 2> cloud_axis_lim( const std::vector<coord_t>& points );
std::vector<coord_t> cloud_bbox( const std::vector<coord_t>& points );

template<typename scalar, int rows, int cols>
inline Eigen::Vector<scalar, cols> dist_euclid( const Eigen::Vector<scalar, rows>& v, const Eigen::Matrix<scalar, rows, cols>& ref )
{
  return ( ref.colwise() - v ).colwise().norm();
}

template<typename scalar, int rows, int cols>
inline Eigen::Vector<scalar, cols> cosine_similarity( const Eigen::Vector<scalar, rows>& v, const Eigen::Matrix<scalar, rows, cols>& ref )
{
  return ( ref.colwise().normalized().transpose() * v.normalized() );
}



class SpatialConstraint
{
  public:
    virtual ~SpatialConstraint() {};
    virtual bool within( const vec_t& ) const = 0;
    virtual bool within( const coord_t& c ) const { return within( vec_from_coord(c) ); }
    virtual bool outside( const vec_t& v ) const { return !within(v); }
    virtual bool outside( const coord_t& c) const { return !within(c); }
    template<typename T>
    std::vector<size_t> get_within( const std::vector<T>& ) const; 

};

class IdentityConstraint : public SpatialConstraint
{
  public:
    IdentityConstraint() {};
    ~IdentityConstraint() = default;
    inline bool within( const vec_t& ) const override { return true; }
};

class ZeroConstraint : public SpatialConstraint
{
  public:
    ZeroConstraint() {};
    ~ZeroConstraint() = default;
    inline bool within( const vec_t& ) const override { return false; }
};

class CollectionConstraint : public SpatialConstraint
{
  public:
    CollectionConstraint( 
      const std::vector<std::shared_ptr<SpatialConstraint>>& constraints_and, const std::vector<std::shared_ptr<SpatialConstraint>>& constraints_or )
     : m_constraints_or(constraints_or), m_constraints_and(constraints_and) 
     {
     }

    ~CollectionConstraint() = default;
    inline bool within( const vec_t& v ) const override 
    {
      auto _lambda_and = [&v]( bool current, const std::shared_ptr<SpatialConstraint>& constr ) -> bool
      { 
        return current && constr->within(v);
      };
      auto _lambda_or = [&v]( bool current, const std::shared_ptr<SpatialConstraint>& constr ) -> bool
      { 
        return current || constr->within(v);
      };
      
      bool out = std::accumulate( m_constraints_and.begin(), m_constraints_and.end(), true, _lambda_and );
      return out && std::accumulate( m_constraints_or.begin(), m_constraints_or.end(), false, _lambda_or );
    }
  
  protected:
    std::vector<std::shared_ptr<SpatialConstraint>> m_constraints_or;
    std::vector<std::shared_ptr<SpatialConstraint>> m_constraints_and;
};

class VoronoiConstraint : public SpatialConstraint
{
  using colvectors_t = Eigen::Matrix<vec_t::Scalar, vec_t::RowsAtCompileTime, Eigen::Dynamic>;
  public:
    VoronoiConstraint( const VoronoiConstraint& other ) : m_attractors(other.m_attractors), m_repulsors(other.m_repulsors) {};
    VoronoiConstraint( const colvectors_t& attractors, const colvectors_t& repulsors  ) : m_repulsors(repulsors), m_attractors(attractors) {};
    VoronoiConstraint( const std::vector<vec_t>& attractors, const std::vector<vec_t>& repulsors );
    VoronoiConstraint( const std::vector<pose_t>& T_attract, const std::vector<pose_t>& T_repulse );
    ~VoronoiConstraint() = default;

    virtual bool within( const vec_t& ) const override;
    static vec_t::Scalar mindist( const vec_t&, const colvectors_t& );
    inline VoronoiConstraint inverse() const { return VoronoiConstraint( m_repulsors, m_attractors ); }
  
  protected:
    colvectors_t m_attractors;
    colvectors_t m_repulsors;
};

class CentersConstraint : public VoronoiConstraint
{
  public:
    using VoronoiConstraint::VoronoiConstraint;
    CentersConstraint( vec_t::Scalar radius, const VoronoiConstraint& other ) : m_radius(radius), VoronoiConstraint(other) {};
    ~CentersConstraint() = default;
    bool within( const vec_t& ) const override;
  
  protected:
    vec_t::Scalar m_radius;
};

class PlanarConstraint : public SpatialConstraint
{
  public:
    virtual ~PlanarConstraint() {};
    bool within( const vec_t& coord ) const override;

  protected:
    std::vector<vec_t> m_normals;
    std::vector<vec_t> m_centers;
};

class OuterSphericalConstraint;
class SphericalConstraint : public SpatialConstraint
{
  public:
    SphericalConstraint( const vec_t& center, double radius ) : m_center(center), m_radius(radius) {};
    ~SphericalConstraint() {};
    bool within( const vec_t& coord ) const override;

  protected:
    vec_t m_center;
    double m_radius;
};

class OuterSphericalConstraint : public SphericalConstraint
{
  public:
    using SphericalConstraint::SphericalConstraint;
    ~OuterSphericalConstraint() {};
    bool within( const vec_t& coord ) const override { return !SphericalConstraint::within(coord); }
};

class PlaneConstraint : public PlanarConstraint
{
  public:
    PlaneConstraint( const vec_t& center, const vec_t& normal );
    ~PlaneConstraint() = default;
    static PlaneConstraint between( const vec_t& a, const vec_t& b, double t );
    PlaneConstraint inverse() const { return PlaneConstraint( m_centers[0], -m_normals[0] ); }

    friend std::ostream& operator<<( std::ostream& o, const PlaneConstraint& p ) 
    { 
      o << "PlaneConstraint( " << "c=(" << p.m_centers[0].transpose() << "); n=(" << p.m_normals[0].transpose() << "); )";
      return o;
    }
};

class FrustumConstraint : public PlanarConstraint
{
  public:
    using SpatialConstraint::within;
    FrustumConstraint( const std::array<vec_t, 4>& near_plane_corners_rhrule, const std::array<vec_t, 4>& far_plane_corners_rhrule );
    ~FrustumConstraint() = default;
    static FrustumConstraint camera_view( double minrange, double maxrange, double w, double h, const pose_t& pose, const intr_t& K );
    static FrustumConstraint cube( const pose_t& T = pose_t::Identity(), double side_length = 1.0 );

    inline const std::vector<vec_t>& corners() const { return m_corners; }
    inline const std::vector<Eigen::Vector3i>& triangles() const { return m_triangles; }
    inline const vec_t& base_normal() const { return m_base_normal; }
    inline const vec_t& base_center() const { return m_base_corner; }

  protected:
    vec_t m_base_corner;
    vec_t m_base_normal;
    std::vector<vec_t> m_corners;
    const std::vector<Eigen::Vector3i> m_triangles 
    {
      { 0, 2, 1 },
      { 0, 5, 4 },
      { 1, 6, 5 },
      { 2, 7, 6 },
      { 3, 0, 7 },
      { 4, 6, 7 }
    };
};


class TimeInterval
{
  public:
    TimeInterval() {};
    TimeInterval( size_t start, size_t stop ) : m_stop(stop), m_start(start) {};
    TimeInterval( const TimeInterval& other ) : m_stop(other.m_stop), m_start(other.m_start) {};

    inline bool within( size_t stamp ) const { return ( m_start <= stamp ) && ( m_stop > stamp ); }
    inline bool overlaps( const TimeInterval& other ) const { return within(other.m_start) || other.within(m_start); }
    inline size_t start() const { return m_start; }
    inline size_t stop() const { return m_stop; }

  protected:
    size_t m_start = 0UL;
    size_t m_stop = -1UL;
};


template<typename T_value>
class DisjointIntervalLookup
{

  public:
    DisjointIntervalLookup() : m_count(0UL) {};
    
    inline void put( size_t start, size_t stop, const T_value& value )
    {
      m_internal[m_count] = value;
      m_start[start] = m_count;
      m_stop[stop] = m_count;
      m_count++;
    }
    
    inline void put( const TimeInterval& interval, const T_value& value )
    {
      put( interval.start(), interval.stop(), value );
    }

    inline std::optional<std::tuple<TimeInterval,T_value>> get( size_t key ) const
    {
      std::optional<std::tuple<TimeInterval,T_value>> out;

      auto start_it = m_start.lower_bound(key);
      auto stop_it = m_stop.upper_bound(key);

      if ( start_it != m_start.begin() && stop_it != m_stop.end() )
      {
        start_it--;
        auto find_start = m_internal.find(start_it->second);
        auto find_stop = m_internal.find(stop_it->second);
        if ( find_start != m_internal.end() && find_start == find_stop )
        {
          out = { TimeInterval( start_it->first, stop_it->first ), m_internal.at(start_it->second) };
        }
      }

      return out;
    }

    inline std::optional<std::tuple<TimeInterval,T_value>> pull( size_t key )
    {
      std::optional<std::tuple<TimeInterval,T_value>> opt_out = get(key);
      if ( opt_out.has_value() )
      {
        auto& [interval, _] = opt_out.value();
        m_internal.erase(m_start[interval.start()]);
        m_start.erase(interval.start());
        m_stop.erase(interval.stop());
      }
      return opt_out;
    }

    inline void print_intervals()
    {
      std::cerr << "all intervals\n";
      auto start_it = m_start.begin();
      auto stop_it = m_stop.begin();
      while( start_it != m_start.end() && stop_it != m_stop.end() )
      {
        std::cerr << "[" + std::to_string( start_it++->first ) + ", " + std::to_string( stop_it++->first ) + "]\n";
      }
    }

    protected:
      size_t m_count;
      std::map<size_t, T_value> m_internal;
      std::map<size_t, size_t> m_start;
      std::map<size_t, size_t> m_stop;

};


}
