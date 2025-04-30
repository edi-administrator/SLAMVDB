#pragma once

#include "resource.hpp"
#include "view.hpp"

#include "config.hxx"
#include "projection.hxx"
#include "model.hpp"

namespace mvdb::opengl
{

class ProgramProjectDistort : public ShaderProgram
{
  public:

    using ShaderProgram::ShaderProgram;

    struct VoxelParams
    {
      float scale = 0.5;
      bool centered = true;
    };

    void set_constant_uniforms( const CameraParams& cparams, const Eigen::Matrix4f& projection_matrix )
    {
      set_uniform_m4f( "u_projection", projection_matrix );
      set_uniform_f( "k1", cparams.k1 );
      set_uniform_f( "k2", cparams.k2 );
      set_uniform_f( "p1", cparams.p1 );
      set_uniform_f( "p2", cparams.p2 );
      set_uniform_f( "_w", cparams.w );
      set_uniform_f( "_h", cparams.h );
      set_uniform_f( "fx", cparams.K(0, 0) );
      set_uniform_f( "fy", cparams.K(1, 1) );
      set_uniform_f( "cx", cparams.K(0, 2) );
      set_uniform_f( "cy", cparams.K(1, 2) );
    }

    void set_view_matrix( const Eigen::Matrix4f& view_matrix )
    {
      set_uniform_m4f( "u_view", view_matrix );
    }

};

class ProgramProjectColor : public ProgramProjectDistort
{
  public:

    using ProgramProjectDistort::ProgramProjectDistort;


    static std::shared_ptr<ProgramProjectColor> make_program()
    {
      auto vpath = get_pkg_share( "shaders/vertex_color.glsl" );
      auto fpath = get_pkg_share( "shaders/fragment_color.glsl" );
      return ShaderProgram::make_program<ProgramProjectColor>( vpath, fpath );
    }

    std::shared_ptr<InstancedModelGeneric> make_model( const std::vector<Eigen::Vector3f>& pos, std::vector<Eigen::Vector3f>& col, const VoxelParams& params )
    {
      auto cube_mesh = InstanceMesh::make_cube( params.scale, params.centered );
      auto model = std::make_shared<InstancedModelGeneric>( cube_mesh, pos );
      model->add_element_values( col, GL_FLOAT, 3 );

      return model;
    }
};

class ProgramProjectIndex : public ProgramProjectDistort
{
  public:

    using ProgramProjectDistort::ProgramProjectDistort;

    static std::shared_ptr<ProgramProjectIndex> make_program()
    {
      auto vpath = get_pkg_share( "shaders/vertex_index.glsl" );
      auto fpath = get_pkg_share( "shaders/fragment_index.glsl" );
      return ShaderProgram::make_program<ProgramProjectIndex>( vpath, fpath );
    }

    std::shared_ptr<InstancedModelGeneric> make_model( const std::vector<Eigen::Vector3f>& pos, std::vector<int32_t>& col, const VoxelParams& params )
    {
      auto cube_mesh = InstanceMesh::make_cube( params.scale, params.centered );
      auto model = std::make_shared<InstancedModelGeneric>( cube_mesh, pos );
      model->add_element_values( col, GL_INT, 1 );

      return model;
    }
};

} // mvdb::opengl