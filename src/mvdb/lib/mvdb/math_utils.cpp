#include "math_utils.hxx"
#include <tuple>
#include <exception>
#include <iostream>

namespace mvdb
{

template<typename T, size_t n>
std::vector<T> vector_from_array( const std::array<T,n>& a )
{
  return std::vector<T> ( a.begin(), a.end() );
}

template<typename T, size_t n>
std::array<T, n> array_from_vector( const std::vector<T>& v )
{
  if ( v.size() != n ) throw std::out_of_range("incorrect vector size conversion to array");
  std::array<T, n> a;
  for ( size_t i = 0; i < a.size(); i++ )
  {
    a[i] = v[i];
  }
  return a;
}

vec_t vec_from_coord( const coord_t& coord )
{
  return { coord[0], coord[1], coord[2] };
}

coord_t coord_from_vec( const vec_t& vec )
{
  return { vec(0), vec(1), vec(2) };
}

vec_t apply_T( const pose_t& T, const vec_t& vec )
{
  return R_from_T(T) * vec + t_from_T(T);
}

coord_t apply_T( const pose_t& T, const coord_t& vec )
{
  return coord_from_vec( apply_T( T, vec_from_coord(vec) ) );
}

std::vector<vec_t> apply_T( const pose_t& T, const std::vector<vec_t>& vec )
{
  std::vector<vec_t> out;
  for ( auto& v : vec )
  {
    out.push_back( apply_T(T, v) );
  }
  return out;
}

std::vector<coord_t> apply_T( const pose_t& T, const std::vector<coord_t>& vec )
{
  std::vector<coord_t> out;
  for ( auto& c : vec )
  {
    out.push_back( apply_T( T, c ) );
  }
  return out;
}

vec_t::Scalar delta_t( const pose_t& a, const pose_t& b )
{
  return t_from_T( a.inverse() * b ).norm();
}

vec_t::Scalar path_length( const std::vector<pose_t>& poses )
{
  vec_t::Scalar out = 0;
  for ( size_t i = 0; i < poses.size() - 1; i++ )
  {
    out += delta_t( poses[i], poses[i+1] );
  }
  return out;
}

pose_t put_t( const pose_t& T, const vec_t& t )
{
  pose_t out = T;
  out.block<3,1>(0,3) = t;
  return out;
}

Eigen::Matrix3d R_from_T( const pose_t& T )
{
  return T.block<3,3>(0,0);
}

vec_t t_from_T( const pose_t& pose )
{
  return pose.block<3,1>(0,3);
}

std::vector<vec_t> t_from_T( const std::vector<pose_t>& T )
{
  std::vector<vec_t> t;
  for ( auto& T_ : T )
  {
    t.push_back( t_from_T(T_) );
  }
  return t;
}

pose_t translate_T( const pose_t& T, const vec_t& delta )
{
  return put_t( T, t_from_T( T ) + delta );
}

pose_t interpolate_pose( const pose_t& a, const pose_t& b, double t_a, double t_b, double t )
{
  // assert( t_b > t_a );
  double _t = ( t - t_a ) / ( t_b - t_a + k_epsilon );

  Eigen::AngleAxis<double> R_a;
  R_a.fromRotationMatrix( a.block<3,3>(0,0) );

  Eigen::AngleAxis<double> R_b;
  R_b.fromRotationMatrix( b.block<3,3>(0,0) );

  Eigen::Quaternion<double> q_a ( R_a );
  Eigen::Quaternion<double> q_b ( R_b );

  Eigen::Quaternion<double> q_t = q_a.slerp( _t, q_b );
  vec_t t_t = a.block<3,1>(0,3) * ( 1 - _t ) + b.block<3,1>(0,3) * _t;

  pose_t out = pose_t::Identity();
  out.block<3,1>(0,3) = t_t;
  out.block<3,3>(0,0) = q_t.toRotationMatrix();

  return out;
}

std::vector<coord_t> box_corners_canonical()
{
  std::array<double, 4> dx { -1, -1, 1, 1 };
  std::array<double, 4> dy { -1, 1, 1, -1 };

  std::vector<coord_t> out;
  for ( double dz = -1; dz < 2; dz += 2 )
  {
    for ( auto i = 0; i < 4; i++ )
    {
      out.push_back( { dx[i], dy[i], dz } );
    }
  }

  return out;
}

std::array<coord_t, 2> cloud_axis_lim( const std::vector<coord_t>& points )
{
  coord_t min, max;
  for ( size_t i = 0; i < 3; i++ )
  {
    min[i] = std::numeric_limits<double>::max();
    max[i] = std::numeric_limits<double>::min();
  }

  for  ( auto& pt : points )
  {
    for ( size_t i = 0; i < 3; i++ )
    {
      min[i] = min[i] < pt[i] ? min[i] : pt[i];
      max[i] = max[i] > pt[i] ? max[i] : pt[i];
    }
  }

  return { min, max };
}

std::vector<coord_t> cloud_bbox( const std::vector<coord_t>& points )
{  
  auto [min, max] = cloud_axis_lim( points );
  auto corners = box_corners_canonical();
  
  for ( auto& corner : corners )
  {
    corner[0] = corner[0] < 0 ? min[0] : max[0];
    corner[1] = corner[1] < 0 ? min[1] : max[1];
    corner[2] = corner[2] < 0 ? min[2] : max[2];
  }

  return corners;
}

template<typename T>
std::vector<size_t> SpatialConstraint::get_within( const std::vector<T>& coords ) const
{
  std::vector<size_t> indices;
  for ( auto i = 0; i < coords.size(); i++ )
  {
    if ( within(coords[i]) ) indices.push_back(i);
  }
  return indices;
}


VoronoiConstraint::VoronoiConstraint( const std::vector<vec_t>& attractors, const std::vector<vec_t>& repulsors ) 
{
  if ( attractors.size() == 0 )
  {
    throw std::out_of_range( "attractors cannot be empty!" );
  }
  if ( repulsors.size() == 0 ) 
  {
    throw std::out_of_range( "repulsors cannot be empty!" );
  }

  m_attractors.resize(vec_t::RowsAtCompileTime, attractors.size());
  m_repulsors.resize(vec_t::RowsAtCompileTime, repulsors.size());

  for ( size_t i = 0; i < attractors.size(); i++ )
  {
    m_attractors.col(i) = attractors[i];
  }

  for ( size_t i = 0; i < repulsors.size(); i++ )
  {
    m_repulsors.col(i) = repulsors[i];
  }
}

VoronoiConstraint::VoronoiConstraint( const std::vector<pose_t>& T_attract, const std::vector<pose_t>& T_repulse )
: VoronoiConstraint( t_from_T(T_attract), t_from_T(T_repulse) )
{
}


vec_t::Scalar VoronoiConstraint::mindist( const vec_t& v, const colvectors_t& m )
{
  Eigen::MatrixXd delta { m.rows(), m.cols() };
  Eigen::VectorXd dist { m.rows() };
  delta = m.colwise() - v;
  dist = delta.colwise().norm();
  return dist.minCoeff();
}

bool VoronoiConstraint::within( const vec_t& coord ) const
{
  return mindist( coord, m_attractors ) < mindist( coord, m_repulsors );
}

bool CentersConstraint::within( const vec_t& vec ) const
{
  return mindist( vec, m_attractors ) < std::min( mindist( vec, m_repulsors ), m_radius );
}

bool PlanarConstraint::within( const vec_t& coord ) const
{
  for ( auto i = 0; i < m_normals.size(); i++ )
  {
    if ( ( coord - m_centers[i] ).dot( m_normals[i] ) < 0 ) return false;
  }
  return true;
}

bool SphericalConstraint::within( const vec_t& coord ) const
{
  return ( (coord - m_center).norm() <= m_radius );
}


PlaneConstraint::PlaneConstraint( const vec_t& center, const vec_t& normal )
{
  m_normals = { normal };
  m_centers = { center };
}

PlaneConstraint PlaneConstraint::between( const vec_t& a, const vec_t& b, double t )
{
  vec_t center = t * b + (1 - t) * a;
  vec_t normal = ( b - a ) / ( b - a ).norm();
  return PlaneConstraint( center, normal );
}


FrustumConstraint::FrustumConstraint( const std::array<vec_t, 4>& near_plane_corners_rhrule, const std::array<vec_t, 4>& far_plane_corners_rhrule )
{
  m_corners.insert( m_corners.end(), near_plane_corners_rhrule.begin(), near_plane_corners_rhrule.end() );
  m_corners.insert( m_corners.end(), far_plane_corners_rhrule.begin(), far_plane_corners_rhrule.end() );

  for ( auto& triangle : m_triangles )
  {
    auto a = m_corners[triangle[0]];
    auto b = m_corners[triangle[1]];
    auto c = m_corners[triangle[2]];

    vec_t normal = ( b - a ).cross( c - a );
    normal /= ( normal.norm() + k_epsilon );

    m_normals.push_back(normal);
    m_centers.push_back(vec_t(a));
  }

  m_base_corner = m_corners[0];
  m_base_normal = m_normals[0];
}

FrustumConstraint FrustumConstraint::camera_view( double minrange, double maxrange, double w, double h, const pose_t& pose, const intr_t& K )
{
  double kx = 1 / K(0,0);
  double ky = 1 / K(1,1);

  std::array<vec_t, 4> near_corners;
  std::array<vec_t, 4> far_corners;
  
  std::array<double, 4> dx { -1, -1, 1, 1 };
  std::array<double, 4> dy { -1, 1, 1, -1 };

  for ( size_t i = 0; i < 4; i++ )
  {

    near_corners[i] = pose.block<3,3>(0,0) * vec_t( dx[i] * w/2 * kx * minrange, dy[i] * h/2 * ky * minrange, minrange ) + pose.block<3,1>(0,3);
    far_corners[i] =  pose.block<3,3>(0,0) * vec_t( dx[i] * w/2 * kx * maxrange, dy[i] * h/2 * ky * maxrange, maxrange ) + pose.block<3,1>(0,3);
  }

  return FrustumConstraint( near_corners, far_corners );
}

FrustumConstraint FrustumConstraint::cube( const pose_t& T, double side_length )
{

  std::array<vec_t, 4> near_corners;
  std::array<vec_t, 4> far_corners;
  
  std::array<double, 4> dx { -1, -1, 1, 1 };
  std::array<double, 4> dy { -1, 1, 1, -1 };

  for ( size_t i = 0; i < 4; i++ )
  {
    near_corners[i] = apply_T( T, vec_t{ dx[i] * side_length / 2, dy[i] * side_length / 2, -side_length / 2 } );
    far_corners[i] = apply_T( T, vec_t{ dx[i] * side_length / 2, dy[i] * side_length / 2, side_length / 2 } );
  }
  
  return FrustumConstraint( near_corners, far_corners );
}

}