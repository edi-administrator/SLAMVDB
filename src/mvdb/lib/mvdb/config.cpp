#include "config.hxx"

namespace mvdb
{

static std::map<std::string, std::string> read_config_txt( const std::string& path )
{
  std::map<std::string, std::string> out;
  std::ifstream f ( path, std::ifstream::in  );

  if ( f.is_open() )
  {
    auto lines = tokenize( f, '\n' );
    for ( auto& line : lines )
    {
      auto tokenized = tokenize( line, '=' );
      if ( tokenized.size() == 2 && tokenized.front().front() != '#' )
      {
        out.insert_or_assign( tokenized.front(), tokenized.back() );
      }
    }
  }

  return out;
}

template<typename T>
static std::optional<T> read_from_config( const std::map<std::string, std::string>& parsed_config, const std::string& key, std::function<T(const std::string&)> cast )
{
  std::optional<T> out;
  auto found = parsed_config.find( key );
  if ( found != parsed_config.end() )
  {
    out = cast( found->second );
  }
  return out;
}

template<typename T>
static void replace_from_config( T& val, const std::map<std::string,std::string>& config, const std::string& key, std::function<T(const std::string&)> cast )
{
  auto opt = read_from_config<T>( config, key, cast );
  if ( opt.has_value() )
  {
    std::cerr << "replacing from config: k: " << key << " v: " << opt.value() << "\n";
    val = opt.value();
  }
}

static size_t cast_ull( const std::string& s )
{
  return std::stoull(s);
}

static double cast_double( const std::string& s )
{
  return std::stod(s);
}

static std::string cast_str( const std::string& s )
{
  return s;  
}

static bool cast_bool( const std::string& s )
{
  if ( s == "true" )
  {
    return true;
  }
  if ( s == "false" )
  {
    return false;
  }
  throw std::out_of_range( "invalid string literal provided for bool cast: " + s );
}

template<typename scalar, int rows, int cols>
static Eigen::Matrix<scalar, rows, cols> cast_to_mat( const std::string& csv )
{
  Eigen::Matrix<scalar,rows,cols> out;
  std::vector<scalar> scalars;

  for ( auto& s : tokenize( csv, ',' ) )
  {
    scalars.push_back( scalar( std::stod(s) ) );
  }

  if ( scalars.size() == rows * cols )
  {
    out = Eigen::Map<Eigen::Matrix<scalar,cols,rows>>( scalars.data() ).transpose();
  }
  else
  {
    throw std::out_of_range( "configuration line wrong size! rows: " + std::to_string(rows) + " cols: " + std::to_string(cols) + " string: '" + csv + "'" );
  }

  return out;
}

std::map<std::string,std::string> maybe_read_file( const std::string& dir, const std::string& filename )
{
  auto dir_iter = std::filesystem::directory_iterator( dir );
  
  auto found = std::find_if( 
    std::filesystem::begin( dir_iter ), 
    std::filesystem::end( dir_iter ), 
    [filename]( const std::filesystem::directory_entry& p ) -> bool 
    { 
      return p.path().filename() == filename;
    }
  );

  std::map<std::string, std::string> out;
  if ( found != std::filesystem::end( dir_iter ) )
  {
    out = read_config_txt( found->path() );
  }

  return out;
}

std::string read_raw_file( const std::string& path )
{
  std::ifstream f ( path, std::ios::in );
  std::stringstream ss;

  if ( !f.is_open() )
  {
    throw std::runtime_error( "[read raw file] could not open file: " + path );
  }
  else
  {
    f.seekg(0);
    f >> ss.rdbuf();
    return std::string( ss.str() );
  }
}

std::string get_pkg_share( const std::string& path )
{
  auto str_share =  std::filesystem::path( ament_index_cpp::get_package_share_directory( "mvdb" ) );
  return ( str_share / path ).string();
}


ConfigReader::ConfigReader( const std::string& root )
{
  auto str_share =  ament_index_cpp::get_package_share_directory(  "mvdb" );
  
  std::cerr << "[ConfigReader] local = " << root << "\n";
  std::cerr << "[ConfigReader] share = " << str_share << "\n";

  m_default = maybe_read_file( str_share + "/config", "mvdb_config.cfg" );
  m_override = maybe_read_file( root, "mvdb_config.cfg" );

  std::cerr << "[ConfigReader] m_default.size() = " << m_default.size() << "\n";
  std::cerr << "[ConfigReader] m_override.size() = " << m_override.size() << "\n";
}

ProjectionParams ConfigReader::get_projection_defaults() const
{
  ProjectionParams p {};

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<size_t>( p.h, config, "projection.h", cast_ull );
    replace_from_config<size_t>( p.w, config, "projection.w", cast_ull );
    replace_from_config<float>( p.near, config, "projection.near", cast_double );
    replace_from_config<float>( p.far, config, "projection.far", cast_double );
    replace_from_config<intr_t>( p.K, config, "projection.K", cast_to_mat<double,3,3> );
    replace_from_config<dstr_t>( p.D, config, "projection.D", cast_to_mat<double,4,1> );
    replace_from_config<pose_t>( p.T_scan_camera, config, "projection.T_scan_camera", cast_to_mat<double,4,4> );
  }

  return p;
}

ScanBufferParams ConfigReader::get_scan_buffer_defaults() const
{
  ScanBufferParams p {};

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<std::string>( p.topic, config, "scan_buffer.topic", cast_str );
    replace_from_config<double>( p.exclusion_radius, config, "scan_buffer.exclusion_radius", cast_double );
    replace_from_config<double>( p.exclusion_phi_start, config, "scan_buffer.exclusion_phi_start", cast_double );
    replace_from_config<double>( p.exclusion_phi_end, config, "scan_buffer.exclusion_phi_end", cast_double );
    replace_from_config<size_t>( p.row_step, config, "scan_buffer.row_step", cast_ull );
    replace_from_config<size_t>( p.col_step, config, "scan_buffer.col_step", cast_ull );
    replace_from_config<bool>( p.is_colmajor, config, "scan_buffer.is_colmajor", cast_bool );
  }

  return p;
}

LocalMapperParams ConfigReader::get_local_mapper_defaults() const
{
  LocalMapperParams p;

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<size_t>( p.tree_params.maxdepth, config, "local_mapper.tree_params.maxdepth", cast_ull );
    replace_from_config<double>( p.tree_params.res, config, "local_mapper.tree_params.res", cast_double );
    replace_from_config<double>( p.tree_params.clear_dist, config, "local_mapper.tree_params.clear_dist", cast_double );
    replace_from_config<size_t>( p.tree_params.downsample, config, "local_mapper.tree_params.downsample", cast_ull );
    replace_from_config<bool>( p.use_quantizer, config, "local_mapper.use_quantizer", cast_bool );
    replace_from_config<size_t>( p.process_n_images, config, "local_mapper.process_n_images", cast_ull );
  }

  return p;
}

GlobalMapperParams ConfigReader::get_global_mapper_defaults() const
{
  GlobalMapperParams p 
  {
    .lookup_params = get_local_voxel_defaults(),
    .global_vox_params = get_global_voxel_defaults(),
  };

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<size_t>( p.tree_params.maxdepth, config, "global_mapper.tree_params.maxdepth", cast_ull );
    replace_from_config<double>( p.tree_params.res, config, "global_mapper.tree_params.res", cast_double );
    replace_from_config<int>( p.planar_map_extent_x, config, "global_mapper.planar_map_extent_x", cast_ull );
    replace_from_config<int>( p.planar_map_extent_y, config, "global_mapper.planar_map_extent_y", cast_ull );
    replace_from_config<std::string>( p.gravity_topic, config, "global_mapper.gravity_topic", cast_str );
    replace_from_config<std::string>( p.floating_frame, config, "global_mapper.floating_frame", cast_str );
    replace_from_config<std::string>( p.gravity_frame, config, "global_mapper.gravity_frame", cast_str );
  }

  return p;
}

LocalVoxelLookupParams ConfigReader::get_local_voxel_defaults() const
{
  LocalVoxelLookupParams p {
    .submap_tree_params = get_local_mapper_defaults().tree_params,
  };

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<double>( p.side_length, config, "local_voxel.side_length", cast_double );
    replace_from_config<size_t>( p.tree_params.maxdepth, config, "local_voxel.tree_params.maxdepth", cast_ull );
    replace_from_config<double>( p.tree_params.res, config, "local_voxel.tree_params.res", cast_double );
  }

  return p;
}

GlobalVoxelMapParams ConfigReader::get_global_voxel_defaults() const
{
  GlobalVoxelMapParams p
  {
    .lookup_params = get_local_voxel_defaults(),
  };
  
  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<size_t>( p.tree_params.maxdepth, config, "global_voxel.tree_params.maxdepth", cast_ull );
    replace_from_config<double>( p.tree_params.res, config, "global_voxel.tree_params.res", cast_double );
  }

  return p;

}


QuantizerParams ConfigReader::get_quantizer_defaults() const
{
  QuantizerParams p {};

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<std::string>( p.discrete_path, config, "quantizer.discrete_path", cast_str );
    replace_from_config<std::string>( p.colors_path, config, "quantizer.colors_path", cast_str );
  }

  return p;
}


PoseBufferParams ConfigReader::get_pose_buffer_params( const std::string& mode ) const
{
  PoseBufferParams p {};

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<std::string>( p.node_name, config, mode + "_pose_buffer.node_name", cast_str );
    replace_from_config<std::string>( p.topic, config, mode + "_pose_buffer.topic", cast_str );
  }

  return p;
}


StaticMapParams ConfigReader::get_static_map_params() const
{
  StaticMapParams p {};

  for ( auto& config : { m_default, m_override } )
  {
    replace_from_config<std::string>( p.points_path, config, "static_map.points_path", cast_str );
    replace_from_config<std::string>( p.poses_path, config, "static_map.poses_path", cast_str );
    replace_from_config<double>( p.tree_params.res, config, "static_map.res", cast_double );
    replace_from_config<size_t>( p.tree_params.maxdepth, config, "static_map.maxdepth", cast_ull );
    replace_from_config<size_t>( p.tree_params.downsample, config, "static_map.downsample", cast_ull );
  }

  return p;
}

}