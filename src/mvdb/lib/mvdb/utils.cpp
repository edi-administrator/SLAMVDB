#include "utils.hxx"

namespace mvdb
{

std::vector<std::pair<size_t, pose_t>> stamped_poses_from_csvs( const std::string& dir, const std::string& ext )
{
  return stamped_mats_from_csvs<4,4>( dir, ext );
}


std::optional<std::vector<coord_t>> coords_from_csv( const std::string& path )
{
  using ret_t = std::vector<coord_t>;
  ret_t rv;
  std::optional<ret_t> ret;
  
  auto opt_vecs = mat_from_csv<3,1>( path );
  if ( opt_vecs.has_value() )
  {
    for ( auto& vec : opt_vecs.value() )
    {
      rv.push_back( coord_t { vec(0), vec(1), vec(2) } );
    }
    ret = rv;
  }

  return ret;
}

std::optional<std::vector<vec_t>> vectors_from_csv( const std::string& path )
{
  return mat_from_csv<3,1>( path );
}

std::optional<std::vector<pose_t>> poses_from_csv( const std::string& path )
{
  return mat_from_csv<4,4>( path );
}

};