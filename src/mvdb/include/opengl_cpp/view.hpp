#pragma once

#include <Eigen/Core>

#include "projection.hxx"

namespace mvdb::opengl
{

struct CameraParams
{
  float k1 = -0.2778251471830247;
  float k2 = 0.0765521162037547;
  float p1 = 0.00038341766757575107;
  float p2 = -0.0002284351390534172;
  
  Eigen::Matrix3f K = Eigen::Matrix3f
  {
    { 1154, 0, 752 / 2 },
    { 0, 1154, 480 / 2 },
    { 0,    0,   1 }
  };

  float w = 752;
  float h = 480;

  static CameraParams from_proj_params( const ProjectionParams& params )
  {
    intr_t K = params.K;
    K.block<2,3>(0,0).array() *= params.supersample;

    auto w_super = float( params.w * params.supersample );
    auto h_super = float( params.h * params.supersample );
    CameraParams p
    {
      .k1 = float(params.D(0)),
      .k2 = float(params.D(1)),
      .p1 = float(params.D(2)),
      .p2 = float(params.D(3)),
      .K = K.cast<float>(),
      .w = w_super,
      .h = h_super
    };

    return p;
  }
};

Eigen::Matrix4f intrinsic_to_projection( Eigen::Matrix3f intrinsic, float width, float height, float near, float far )
{
  auto fx = intrinsic(0, 0);
  auto fy = intrinsic(1, 1);
  auto cx = intrinsic(0, 2);
  auto cy = intrinsic(1, 2);

  Eigen::Matrix4f projection_matrix
  {
    { 2 * fx / width,   0,                -1 * ( 2 * cx / width - 1 ),      0                                 },
    { 0,                -2 * fy / height, -1 * ( 2 * cy / height - 1),             0                          },
    { 0,                0,                -( far + near ) / (far - near ),  -2 * far * near / ( far - near )  },
    { 0,                0,                -1,                               0                                 },
  };

  return projection_matrix;
}

Eigen::Matrix4f intrinsic_to_projection( const CameraParams& cparams, float near, float far )
{
  return intrinsic_to_projection( cparams.K, cparams.w, cparams.h, near, far );
}
  
}
