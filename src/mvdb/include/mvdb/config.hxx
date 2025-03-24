#pragma once

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "octree.hxx"
#include "record.hxx"
#include "projection.hxx"
#include "buffer_nodes.hxx"
#include "mapper_nodes.hxx"
#include "utils.hxx"

namespace mvdb
{

class ConfigReader
{
  public:

    ConfigReader( const std::string& root = std::filesystem::current_path() );

    ProjectionParams get_projection_defaults() const;
    LocalMapperParams get_local_mapper_defaults() const;
    GlobalMapperParams get_global_mapper_defaults() const;
    LocalVoxelLookupParams get_local_voxel_defaults() const;
    GlobalVoxelMapParams get_global_voxel_defaults() const;
    QuantizerParams get_quantizer_defaults() const;
    PoseBufferParams get_pose_buffer_params( const std::string& mode = "tracker" ) const;
  
  protected:
    std::map<std::string, std::string> m_default;
    std::map<std::string, std::string> m_override;
};

}