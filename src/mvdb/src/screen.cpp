#include <iostream>
#include <memory>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "resource.hpp"
#include "model.hpp"
#include "renderer.hpp"
#include "view.hpp"
#include "detail.hpp"


using namespace mvdb::opengl;

void project_points_screen( std::vector<Eigen::Vector3f>& positions, std::vector<Eigen::Vector3f>& colors )
{
  GlfwStateWrapper wrp { GlfwStateWrapper::Params { .should_show = true } };
  CameraParams cparams {};

  Eigen::Matrix4f projection_matrix = intrinsic_to_projection( cparams, 0.1, 50 );
  Eigen::Isometry3f cpose = Eigen::Isometry3f::Identity();
  cpose =  cpose.pretranslate( Eigen::Vector3f { 0, 0, 10 } );

  auto shader_program = ProgramProjectColor::make_program();
  auto model = shader_program->make_model( positions, colors, ProgramProjectDistort::VoxelParams { .scale = 0.25, .centered = true } );

  auto shader_binding = shader_program->get_bind();
  shader_program->set_constant_uniforms( cparams, projection_matrix );

  float rot = 0;
  float delta_rot = 0.01;

  while ( !wrp.should_close() )
  {
    rot += delta_rot;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = Eigen::AngleAxis<float>( rot, Eigen::Vector3f::UnitY() ).matrix();

    shader_program->set_view_matrix( ( T * cpose.matrix() ).inverse() );

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    model->draw();

    wrp.update();  
  }

}

int main()
{
  std::cout << "builds, works and prints" << std::endl;

  std::vector<Eigen::Vector3f> positions 
  {
    { 0, 0, 0 },
    { 1, 1, -1 },
    { 1, -1, -1 },
    { -1, 1, -1 },
    { -1, -1, -1 },
    { 1, 1, 1 },
    { 1, -1, 1 },
    { -1, 1, 1 },
    { -1, -1, 1 },
  };

  std::vector<Eigen::Vector3f> colors 
  {
    { 1, 1, 1 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 0.5, 0, 0.5 },
    { 1, 0, 0 },
    { 0, 1, 0 },
    { 0, 0, 1 },
    { 0.5, 0, 0.5 },
  };

  project_points_screen( positions, colors );
}