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

std::vector<vec_t> vectors_from_bin( const std::string& path )
{
  auto buffer = load_bin<vec_t::Scalar>( path );

  if ( buffer.size() % 3 != 0 )
  {
    throw std::runtime_error( "vector file content size is not divisible by 3! path = " + path );
  }
  std::vector<vec_t> out;
  
  for ( size_t i = 0; i < buffer.size(); i += 3)
  {
    vec_t line = Eigen::Map<vec_t>( buffer.data() + i );
    out.push_back( line );
  }
  
  return out;
}

std::vector<coord_t> coords_from_bin( const std::string& path )
{
  std::vector<coord_t> out;
  for ( auto& v : vectors_from_bin( path ) )
  {
    out.push_back( coord_from_vec( v ) );
  }
  return out;
}

std::optional<std::vector<pose_t>> poses_from_csv( const std::string& path )
{
  return mat_from_csv<4,4>( path );
}

std::string strip_leading_whitespace( const std::string& s )
{
  auto first_not_space = std::find_if( s.begin(), s.end(), []( const char c ) { return !std::isspace(c); } );
  return std::string { first_not_space, s.end() }; 
}

std::string strip_trailing_whitespace( const std::string& s )
{
  auto last_not_space = std::find_if( s.rbegin(), s.rend(), []( const char c ) { return !std::isspace(c); } );
  return std::string { s.begin(), last_not_space.base() };
}

std::string strip( const std::string& s )
{
  return strip_leading_whitespace( strip_trailing_whitespace( s ) );
}

std::vector<std::string> tokenize( std::istream& istream, const char delim )
{
  std::string token;
  std::vector<std::string> out;
  while ( std::getline( istream, token, delim ) )
  {
    out.push_back( strip( token ) );
  }
  return out;

}

std::vector<std::string> tokenize( const std::string& str, const char delim )
{
  std::stringstream ss ( str );
  return tokenize( ss, delim );
}

size_t stamp_from_path( const std::filesystem::path& path )
{
  auto tokens = tokenize( path.filename().stem().string(), '_' );
  return std::stoul( tokens.back() );
}


bool path_stamp_comp( const std::filesystem::path& a, const std::filesystem::path& b )
{
  return stamp_from_path(a) < stamp_from_path(b);
}


};