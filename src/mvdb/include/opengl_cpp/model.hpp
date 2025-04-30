#pragma once

#include <Eigen/Core>
#include <GL/glew.h>
#include <memory>
#include <vector>

#include "resource.hpp"

namespace mvdb::opengl
{
  
class InstanceMesh
{
  public:

    InstanceMesh( const std::vector<float>& vertices, const std::vector<uint>& triangles )
    : m_triangles( triangles ), m_vertices( vertices )
    {
    }

    static std::shared_ptr<InstanceMesh> make_cube( float scale = 0.125, bool centered = true )
    {
      std::vector<Eigen::Vector3f> corners;
      corners.push_back( { 0, 0, 0 } ); // # a
      corners.push_back( { 0, 1, 0 } ); // # b
      corners.push_back( { 1, 1, 0 } ); // # c
      corners.push_back( { 1, 0, 0 } ); // # d
      corners.push_back( { 0, 0, 1 } ); // # e
      corners.push_back( { 0, 1, 1 } ); // # f
      corners.push_back( { 1, 1, 1 } ); // # g
      corners.push_back( { 1, 0, 1 } ); // # h
    
      for ( auto& corner : corners )
      {
        if ( centered )
        {
          corner.array() -= 0.5;
        }
        corner *= scale;
      }
    
      std::vector<Eigen::Vector3i> triangles;
      triangles.push_back( { 0, 3, 1 } ); // # a d b
      triangles.push_back( { 2, 1, 3 } ); // # c b d
      triangles.push_back( { 4, 5, 7 } ); // # e f h
      triangles.push_back( { 6, 7, 5 } ); // # g h f
      triangles.push_back( { 3, 0, 7 } ); // # d a h
      triangles.push_back( { 4, 7, 0 } ); // # e h a
      triangles.push_back( { 2, 6, 1 } ); // # c g b
      triangles.push_back( { 5, 1, 6 } ); // # f b g
      triangles.push_back( { 0, 1, 4 } ); // # a b e
      triangles.push_back( { 5, 4, 1 } ); // # f e b
      triangles.push_back( { 3, 7, 2 } ); // # d h c
      triangles.push_back( { 6, 2, 7 } ); // # g c h
    
      std::vector<float> _corners;
      std::vector<uint> _triangles;
    
      for ( auto& corner : corners )
      {
        _corners.push_back( corner(0) );
        _corners.push_back( corner(1) );
        _corners.push_back( corner(2) );
      }
    
      for ( auto& triangle : triangles )
      {
        _triangles.push_back( triangle(0) );
        _triangles.push_back( triangle(1) );
        _triangles.push_back( triangle(2) );
      } 
    
      return std::make_shared<InstanceMesh>( _corners, _triangles );
    }

    std::vector<float>& vertices() { return m_vertices; }
    std::vector<uint>& triangles() { return m_triangles; }

  protected:
    std::vector<uint> m_triangles;
    std::vector<float> m_vertices;

};

class InstancedModelGeneric
{
  public:
  
    InstancedModelGeneric( std::shared_ptr<InstanceMesh> instance, const std::vector<Eigen::Vector3f>& pos )
    : m_instance( instance ), m_positions( pos )
    {
      m_vao = std::make_unique<VertexArrayObject>();

      m_vao->make_element_buffer( m_instance->triangles() );

      m_vao->add_array_buffer( 0, m_instance->vertices(), 3, sizeof(float) * 3, NULL );
      m_vao->add_array_buffer( 1, m_positions, 3, sizeof(Eigen::Vector3f), 1 );
    }

    template<typename T = Eigen::Vector3f>
    void add_element_values( std::vector<T>& values, GLenum itype, size_t dim )
    {

      m_vao->add_array_buffer<T>( 2, values, dim, sizeof(T), 1, itype );
    }

    void draw()
    {
      auto bind = m_vao->get_bind();

      glDrawElementsInstanced(
        GL_TRIANGLES,
        m_instance->triangles().size(),
        GL_UNSIGNED_INT,
        NULL, // this is a position offset in the EBO
        m_positions.size()
      );
    }

  protected:
    std::shared_ptr<InstanceMesh> m_instance;
    std::vector<Eigen::Vector3f> m_positions;
    std::unique_ptr<VertexArrayObject> m_vao;

};


} // mvdb::opengl