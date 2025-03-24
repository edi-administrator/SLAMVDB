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
  intr_t K = intr_t::Identity();
  pose_t T_scan_camera = pose_t::Identity();
};

class ProjectionBuffer
{
  public:

    ProjectionBuffer( size_t w, size_t h, const Eigen::Matrix3d& K, const Eigen::Matrix4d& T );
    ProjectionBuffer( const Eigen::Matrix4d& T, const ProjectionParams& params = ProjectionParams {} ) : ProjectionBuffer( params.w, params.h, params.K, T * params.T_scan_camera ) {};
    void project( Octree& tree );
    void fill_sem( const std::vector<sem_t>& values );

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

}