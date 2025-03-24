#pragma once

#include <open3d/Open3D.h>
#include "math_utils.hxx"
#include "utils.hxx"
#include "octree.hxx"
#include "record.hxx"
#include <chrono>

namespace mvdb
{

inline void mesh_box( std::vector<Eigen::Vector3d>& corners, std::vector<Eigen::Vector3i>& edges, Eigen::Vector3d xyz, double res, size_t idx )
{
  static std::vector<Eigen::Vector3i> _edges {
    {0, 2, 6}, // RIGHT HAND RULE ORDER MUST BE OBSERVED
    {0, 6, 4},
    {0, 3, 2},
    {0, 1, 3},
    {0, 4, 5},
    {0, 5, 1},
    {2, 7, 6},
    {2, 3, 7},
    {4, 6, 7},
    {4, 7, 5},
    {1, 7, 3},
    {1, 5, 7}
  };

  for ( double dx = -1;  dx < 2; dx += 2 )
  {
    for ( double dy = -1;  dy < 2; dy += 2 )
    {
      for ( double dz = -1;  dz < 2; dz += 2 )
      {
        corners.push_back( xyz + Eigen::Vector3d( dx, dy, dz ) * res * 0.4 );
      }

    }
  }

  for ( auto i = 0; i < _edges.size(); i++ )
  {
    edges.push_back(Eigen::Vector3i(_edges[i]));
    for ( auto& s : edges[edges.size() - 1] )
    {
      s += idx;
    }
  }

}

inline std::shared_ptr<open3d::geometry::TriangleMesh> tree_mesh( 
  mvdb::Octree& tree, 
  std::function<bool(mvdb::OctreeNode&)> predicate = []( mvdb::OctreeNode& ){ return true; },
  std::function<void(OctreeNode&,std::vector<vec_t>&)> colorkey = [](OctreeNode&n, std::vector<vec_t>& out) -> void
  {
    auto& ln = static_cast<mvdb::OctreeLeafOccNode&>(n);
    out.push_back( { (1 - ln.prob()) * 1.8, (1 - ln.prob()) * 1.8, (1 - ln.prob()) * 1.8  } );
  }
)
{
  std::vector<mvdb::vec_t> vertices;
  std::vector<Eigen::Vector3i> triangles;
  std::vector<Eigen::Vector3d> colors;

  size_t idx = 0;
  size_t color_idx = 0;

  auto callback = [&](mvdb::OctreeNode& n){
    if ( n.is_leaf() )
    {
      auto coord = n.coord();
      mesh_box( vertices, triangles, {coord[0],coord[1],coord[2]}, tree.resolution(), idx);
      idx = vertices.size();
      for ( color_idx; color_idx < vertices.size(); color_idx++ )
      {
        colorkey(n, colors);
      }
    }
  };
  tree.traverse(callback, predicate);

  auto mesh = std::make_shared<open3d::geometry::TriangleMesh>(vertices, triangles);
  mesh->vertex_colors_ = colors;
  mesh->ComputeVertexNormals();
  return mesh;
}

inline std::shared_ptr<open3d::geometry::TriangleMesh> submap_mesh( 
  Submap& submap, bool use_dot = false, const sem_t& query = sem_t::Zero(), std::function<bool(mvdb::OctreeNode&)> predicate = []( mvdb::OctreeNode& ){ return true; } )
{
  size_t ns_total = 0;
  sem_t normalized_query = query / ( query.norm() + k_epsilon );

  auto colorkey = [&]( OctreeNode& n, std::vector<vec_t>& out )
  {
    auto& ln = static_cast<OctreeLeafOccNode&>(n);
    auto uid = submap.spatial_idx().at( n.coord() );
    if ( uid.has_value() )
    {
      sem_t sem = submap.global_idx().at( uid.value() )->value;
      if ( use_dot )
      {
        sem /= ( sem.norm() + k_epsilon );
        sem_t::Scalar sim = pow( sem.dot( query ) / 2 + 0.5, 2 );
        out.push_back( { sim, sim, sim } );
      }
      else
      {
        out.push_back( { sem(0), sem(1), sem(2) } );
      }
    }
    else
    {
      out.push_back( { (1 - ln.prob()) * 1.8, (1 - ln.prob()) * 1.8, (1 - ln.prob()) * 1.8  } );
    }
  };

  auto t1 = std::chrono::system_clock().now();
  auto m = tree_mesh( submap.tree(), predicate, colorkey );
  auto t2 = std::chrono::system_clock().now();
  std::cout << "count total: " << double((t2 - t1).count()) / 1e9 << "\n";
  m->Transform(submap.pose());
  return m;
}

inline std::shared_ptr<open3d::geometry::PointCloud> pose_cloud( const pose_t& pose, double scale = 1., double steps = 10 )
{
  std::vector<vec_t> pts;
  std::vector<vec_t> colors;


  for ( auto axis = 0; axis < 3; axis++ )
  {
    vec_t vbase = { 0, 0, 0 };
    vec_t cbase = { 0, 0, 0 };
    vbase(axis) = 1;
    cbase(axis) = 0.99;
    for ( double da = 0; da < scale; da+= scale/steps )
    {
      auto v = vbase * da;
      pts.push_back( pose.block<3,3>(0,0) * v + pose.block<3,1>(0,3) );
      colors.push_back( vec_t(cbase) );
    }
  }
  auto pcd = std::make_shared<open3d::geometry::PointCloud>(pts);
  pcd->colors_ = colors;
  return pcd;
}

inline void pose_cloud( std::vector<coord_t>& out_coord, std::vector<double>& out_color, const pose_t& pose, double scale = 1., double steps = 10 )
{
  std::array<double, 3> _colors { 0.25, 0.5, 0.75 };
  for ( auto axis = 0; axis < 3; axis++ )
  {
    vec_t vbase = { 0, 0, 0 };
    vec_t cbase = { 0, 0, 0 };
    vbase(axis) = 1;
    cbase(axis) = 0.99;
    for ( double da = 0; da < scale; da+= scale/steps )
    {
      auto v = vbase * da;
      out_coord.push_back(  coord_from_vec( pose.block<3,3>(0,0) * v + pose.block<3,1>(0,3) ) );
      out_color.push_back( _colors[axis] );
    }
  }
}

inline std::shared_ptr<open3d::geometry::TriangleMesh> frustum_mesh( const FrustumConstraint f )
{
  auto m = std::make_shared<open3d::geometry::TriangleMesh>( f.corners(), f.triangles() );
  m->ComputeVertexNormals();
  return m;
}


}