#pragma once

#include "octree.hxx"

namespace mvdb
{

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

}