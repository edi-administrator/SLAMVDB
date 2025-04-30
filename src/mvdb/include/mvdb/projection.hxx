#pragma once

#include <tuple>
#include <unordered_map>
#include "math_utils.hxx"
#include "octree.hxx"

namespace mvdb
{

class Cube
{
  public:
    Cube( double edge_length ) { m_corners = m_corner_order * edge_length / 2; }
    std::vector<std::tuple<pxi_t, double, bool>> pixel_projection( vec_t center_coord, intr_t K, pose_t T_map_camera );

  protected:

    inline static const Eigen::Matrix<double, 8, 3> m_corner_order
    {
      {-1, -1, -1}, // 0
      { 1, -1, -1}, // 1
      { 1,  1, -1}, // 2
      {-1,  1, -1}, // 3
      {-1, -1,  1}, // 4
      { 1, -1,  1}, // 5
      { 1,  1,  1}, // 6
      {-1,  1,  1}  // 7
    };
    double m_len;
    Eigen::Matrix<double, 8, 3> m_corners;

};

struct ProjectionParams
{
  size_t w = 100;
  size_t h = 100;
  size_t supersample = 1;
  float near = 0.1;
  float far = 20;
  intr_t K = intr_t::Identity();
  dstr_t D = dstr_t::Zero();
  pose_t T_scan_camera = pose_t::Identity();
};

struct ProjectionResult
{
  std::vector<sem_t> sem;
  std::vector<size_t> counts;
  std::vector<xkey_t> keys;
  std::vector<nano_t> times;
};

class ProjectionBuffer
{
  public:

    ProjectionBuffer( size_t w, size_t h, const Eigen::Matrix3d& K, const Eigen::Matrix4d& T );
    ProjectionBuffer( const Eigen::Matrix4d& T, const ProjectionParams& params = ProjectionParams {} ) : ProjectionBuffer( params.w, params.h, params.K, T * params.T_scan_camera ) {};
    void project( Octree& tree );
    void project( const std::vector<coord_t>& tree, const OctreeParams& params );
    void fill_sem( const std::vector<sem_t>& values );
    ProjectionResult put_sem( const std::vector<sem_t>& values );
    void put_sem( const std::vector<sem_t>& values, ProjectionResult& result );

    inline const std::vector<sem_t>& sem() const { return m_sem; }
    inline const std::vector<size_t>& counts() const { return m_counts; } 
    inline const std::vector<xkey_t>& keys() const { return m_keys; } 
    inline const std::vector<size_t>& times() const { return m_times; }

  protected:
    pose_t m_Twc;
    intr_t m_K;
    size_t m_w, m_h;

    std::vector<sem_t> m_sem;
    std::vector<size_t> m_counts;
    std::vector<xkey_t> m_keys;
    std::vector<size_t> m_times;

    FrustumConstraint m_frustum;
    std::unordered_map<size_t, xkey_t> m_closest_key;
    std::unordered_map<size_t, double> m_z;
};

struct RenderingRequest
{
  OctreeParams tree_params;
  std::vector<coord_t> points;
  std::vector<pose_t> camera_poses;
  std::vector<std::shared_ptr<std::vector<sem_t>>> images;
};

class IProjector
{
  public:
    virtual ~IProjector() = default;
    virtual ProjectionResult render( std::shared_ptr<const RenderingRequest> request ) = 0;
};


class CPUProjectorWrapper : public IProjector
{
  public:
    ~CPUProjectorWrapper() = default;
    CPUProjectorWrapper( const ProjectionParams& params = ProjectionParams {} )
    : m_params( params )
    {
    }

    virtual ProjectionResult render( std::shared_ptr<const RenderingRequest> request ) override
    {
      ProjectionResult out;
      size_t step = std::max( 1UL, request->camera_poses.size() / 8UL );
      for ( size_t i = 0; i < request->camera_poses.size(); i += step )
      {
        ProjectionBuffer pbuf ( request->camera_poses[i], m_params );
        pbuf.project( request->points, request->tree_params );
        pbuf.put_sem( *request->images[i], out );
      }
      return out;
    }
  
  protected:
    ProjectionParams m_params;
    OctreeParams m_tree_params;
    std::vector<coord_t> m_points;

};

}