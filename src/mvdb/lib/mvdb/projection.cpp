#include "projection.hxx"
#include <exception>

namespace mvdb
{


std::vector<std::tuple<pxi_t, double, bool>> Cube::pixel_projection( vec_t center_coord, intr_t K, pose_t T_map_camera )
{
  std::vector<std::tuple<pxi_t, double, bool>> pixels_depth;
  // double z = center_coord.norm();

  pose_t T_camera_map = T_map_camera.inverse();

  double z = apply_T( T_camera_map, center_coord ).z();

  Eigen::Matrix<double, 8, 3> corners = m_corners.array().rowwise() + center_coord.array().transpose();
  Eigen::Matrix<double, 3, 8> pts_camera_frame = ( T_camera_map.block<3,3>(0,0) * corners.transpose() ).array().colwise() + T_camera_map.block<3,1>(0,3).array();
  Eigen::Matrix<double, 3, 8> projected = K * pts_camera_frame;
  Eigen::Matrix<double, 2, 8> dehomogenized = projected.block<2,8>(0,0).array().rowwise() / projected.block<1,8>(2,0).array();

  pxf_t min_corner, max_corner;
  for ( auto dim = 0; dim < 2; dim++ )
  {
    min_corner[dim] = dehomogenized.block<1,8>(dim, 0).minCoeff();
    max_corner[dim] = dehomogenized.block<1,8>(dim, 0).maxCoeff();
  }

  pxf_t center = ( max_corner + min_corner ) / 2;
  double radius = ( max_corner - min_corner ).norm() / sqrt(2);
  double radius_vis = radius;

  for ( int i = min_corner[0]; i < max_corner[0]; i++ )
  {
    double y_expr_neg = -sqrt( pow(radius, 2) - pow( i - center[0], 2 ) ) + center[1];
    double y_expr_pos = sqrt( pow(radius, 2) - pow( i - center[0], 2 ) ) + center[1];
    for ( int j = floor( y_expr_neg ); j < ceil( y_expr_pos ); j++ )
    {
      pxf_t pt = { i - center[0], j - center[1] }; 
      pixels_depth.push_back( { pxi_t(i,j), z, ( pt.norm() <= radius_vis ) } );
    }
  }

  return pixels_depth;
}


ProjectionBuffer::ProjectionBuffer( size_t w, size_t h, const Eigen::Matrix3d& K, const Eigen::Matrix4d& T )
: m_frustum( FrustumConstraint::camera_view( 1, 50, w, h, T, K )), m_Twc(T), m_K(K), m_w(w), m_h(h)
{
}

void ProjectionBuffer::project( Octree& tree )
{
  Cube cube { tree.resolution() };
  auto pts_in_view = tree.filtered_pts( m_frustum );

  for ( auto& pt : pts_in_view )
  {
    auto covered_px = cube.pixel_projection( vec_from_coord(pt), m_K, m_Twc );

    for ( auto& cvpx : covered_px )
    {
      auto& ij = std::get<0>(cvpx);
      auto& z = std::get<1>(cvpx);
      bool is_visible = std::get<2>(cvpx);

      if ( is_visible )
      {
        xkey_t xkey = tree.coord_to_xkey(pt);
        size_t map_key = ij[1] * m_w + ij[0]; // x <-> j; y <-> i;
        
        if ( m_closest_key.find( map_key ) == m_closest_key.end() || m_z.at(map_key) >= z )
        {
          m_closest_key.insert_or_assign(map_key, xkey);
          m_z.insert_or_assign(map_key, z);
        }
      }
    }
  }

}

void ProjectionBuffer::project( const std::vector<coord_t>& points, const OctreeParams& params )
{
  Cube cube { params.res };
  auto pts_in_view = m_frustum.get_within( points );

  for ( auto& pt : pts_in_view )
  {
    auto covered_px = cube.pixel_projection( vec_from_coord( pt ), m_K, m_Twc );

    for ( auto& cvpx : covered_px )
    {
      auto& ij = std::get<0>(cvpx);
      auto& z = std::get<1>(cvpx);
      bool is_visible = std::get<2>(cvpx);

      if ( is_visible )
      {
        xkey_t xkey = params.coord_to_xkey_leaf( pt );
        size_t map_key = ij[1] * m_w + ij[0]; // x <-> j; y <-> i;
        
        if ( m_closest_key.find( map_key ) == m_closest_key.end() || m_z.at(map_key) >= z )
        {
          m_closest_key.insert_or_assign(map_key, xkey);
          m_z.insert_or_assign(map_key, z);
        }
      }
    }
  }

}

void ProjectionBuffer::fill_sem( const std::vector<sem_t>& values )
{
  if ( values.size() != m_h * m_w )
  {
    throw std::range_error("Can't use the semantic vector! Size = " + std::to_string(m_sem.size()) + " does not equal h*w = " \
    + std::to_string(m_h*m_w));
  }

  for ( auto i = 0; i < values.size(); i++ )
  {
    auto it = m_closest_key.find( i );
    if ( it != m_closest_key.end() )
    {
      m_sem.push_back(values[i]);
      m_keys.push_back((*it).second);
      m_counts.push_back(1);
      m_times.push_back(-1);
    }
  }
}


ProjectionResult ProjectionBuffer::put_sem( const std::vector<sem_t>& values )
{
  ProjectionResult result;

  if ( values.size() != m_h * m_w )
  {
    throw std::range_error( "Can't use the semantic vector! Size = " + std::to_string(m_sem.size()) + " does not equal h*w = " \
    + std::to_string(m_h*m_w) );
  }

  for ( auto i = 0; i < values.size(); i++ )
  {
    auto it = m_closest_key.find( i );
    if ( it != m_closest_key.end() )
    {
      result.sem.push_back(values[i]);
      result.keys.push_back((*it).second);
      result.counts.push_back(1);
      result.times.push_back(-1);
    }
  }

  return result;
}

void ProjectionBuffer::put_sem( const std::vector<sem_t>& values, ProjectionResult& result )
{
  if ( values.size() != m_h * m_w )
  {
    throw std::range_error( "Can't use the semantic vector! Size = " + std::to_string(m_sem.size()) + " does not equal h*w = " \
    + std::to_string(m_h*m_w) );
  }

  for ( auto i = 0; i < values.size(); i++ )
  {
    auto it = m_closest_key.find( i );
    if ( it != m_closest_key.end() )
    {
      result.sem.push_back(values[i]);
      result.keys.push_back((*it).second);
      result.counts.push_back(1);
      result.times.push_back(-1);
    }
  }
}

}