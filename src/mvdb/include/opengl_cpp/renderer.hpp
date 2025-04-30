#pragma once

#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace mvdb::opengl
{

class GlfwStateWrapper
{
  public:

    struct Params
    {
      size_t w = 752;
      size_t h = 480;
      bool should_show = false;
    };

    GlfwStateWrapper( const Params& params )
    {
      auto initialized = glfwInit();

      if ( !initialized )
      {        
        throw std::runtime_error( "could not initialize glfw!" );
      }

      if ( params.should_show )
      {
        glfwWindowHint( GLFW_VISIBLE, GLFW_TRUE );
      }
      else
      {
        glfwWindowHint( GLFW_VISIBLE, GLFW_FALSE );
      }

      glfwWindowHint( GLFW_DEPTH_BITS, 32);
      
      m_window = glfwCreateWindow( params.w, params.h, "Test Window", NULL, NULL );

      if ( !m_window )
      {
        throw std::runtime_error( "could not create window!" );
      }

      glfwMakeContextCurrent( m_window );
      glEnable( GL_DEPTH_TEST ); // set after making the window context current
      
      glewInit();      

      std::string context = "OpenGL context:\n";

      context += "vendor:  " + std::string( (char*) glGetString( GL_VENDOR ) ) + "\n";
      context += "renderer:  " + std::string( (char*) glGetString( GL_RENDERER ) ) + "\n";
      context += "version:  " + std::string( (char*) glGetString( GL_VERSION ) ) + "\n";
      
      std::cerr << context << std::endl;
    }

    ~GlfwStateWrapper()
    {
      std::cerr << "terminating glfw!" << std::endl;
      glfwTerminate();
    }

    bool should_close()
    {
      return glfwWindowShouldClose( m_window );
    }

    void update()
    {
      glfwSwapBuffers( m_window );
      glfwPollEvents();
    }
  
  protected:
    GLFWwindow* m_window;
};

} // mvdb::opengl