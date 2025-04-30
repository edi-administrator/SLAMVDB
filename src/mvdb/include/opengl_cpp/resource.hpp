#pragma once

#include <Eigen/Core>
#include <GL/glew.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <stdexcept>
#include <iostream>
#include <fstream>

namespace mvdb::opengl
{

std::string read_file( const std::string& path )
{
  std::ifstream f ( path, std::ios::in | std::ios::ate ); // ate sets the stream pos at the end
  std::stringstream ss;

  if ( !f.is_open() )
  {
    throw std::runtime_error( "could not open file: " + path );
  }
  else
  {
    f.seekg(0);
    f >> ss.rdbuf();
    return std::string( ss.str() );
  }
}

class BindingGL;

class IGlResource
{
  public:
    virtual ~IGlResource() = default;
    virtual void bind() = 0;
    virtual void unbind() = 0;
    BindingGL get_bind();

    GLuint handle() const { return m_handle; }

  protected:
    GLuint m_handle;
};

class BindingGL
{
  public:
    BindingGL( IGlResource& resource ) : m_resource( resource ) { resource.bind(); }
    ~BindingGL() { m_resource.unbind(); }

  protected:
    IGlResource& m_resource;
};

BindingGL IGlResource::get_bind()
{
  return BindingGL( *this );
}

class AbstractGLBuffer : public IGlResource
{
  public:
    AbstractGLBuffer( GLuint buffer_type )
    : m_buffer_type( buffer_type )
    {
      glGenBuffers( 1, &m_handle );
    }

    ~AbstractGLBuffer()
    {
      glDeleteBuffers( 1, &m_handle );
    }

    void put_data( size_t size, void* data )
    {
      auto b = get_bind();
      glBufferData( m_buffer_type, size, data, GL_STATIC_DRAW );
    }

    template<typename T>
    void put_vector_data( std::vector<T>& data )
    {
      put_data( sizeof(T) * data.size(), reinterpret_cast<void*>( data.data() ) );
    }

    void set_location( size_t idx, size_t dim, size_t stride, GLenum _type = GL_FLOAT )
    {
      auto b = get_bind();
      
      if ( _type == GL_FLOAT )
      {
        glVertexAttribPointer( idx, dim, _type, GL_FALSE, stride, NULL );
      }
      else if ( _type == GL_INT )
      {
        glVertexAttribIPointer( idx, dim, _type, stride, NULL );
      }

      m_has_location = true;
      m_location = idx;
    }
    
    void set_divisor( size_t divisor )
    {
      if ( !m_has_location )
      {
        throw std::logic_error( "buffer has no location set!" );
      }

      auto b = get_bind();
      glVertexAttribDivisor( m_location, divisor );
    }

    void bind() override
    {
      glBindBuffer( m_buffer_type, m_handle );
    }

    void unbind() override
    {
      glBindBuffer( m_buffer_type, 0 );
    }


  protected:
    GLuint m_buffer_type;
    GLuint m_location = -1;
    bool m_has_location = false;
    bool m_has_divisor = false;


};


class ELementArrayBuffer : public AbstractGLBuffer
{
  public:
    ~ELementArrayBuffer() = default;
    ELementArrayBuffer() : AbstractGLBuffer( GL_ELEMENT_ARRAY_BUFFER ) {};


};

class ArrayBuffer : public AbstractGLBuffer
{
  public:
    ~ArrayBuffer() = default;
    ArrayBuffer() : AbstractGLBuffer( GL_ARRAY_BUFFER ) {};
    GLuint location() const { return m_location; }

};

class VertexArrayObject : public IGlResource
{
  public:

    VertexArrayObject()
    {
      glGenVertexArrays( 1, &m_handle );
      // std::cerr << "created vao: " << m_handle << std::endl;
    }

    ~VertexArrayObject()
    {
      // std::cerr << "deleting vao: " << m_handle << std::endl;
      glDeleteVertexArrays( 1, &m_handle );
    }

    template<typename T>
    void make_element_buffer( std::vector<T>& data )
    {
      auto l = get_bind();

      m_element_buffer = std::make_unique<ELementArrayBuffer>();
      m_element_buffer->put_vector_data( data );
    }

    template<typename T>
    void add_array_buffer( size_t loc, std::vector<T>& data, size_t dim, size_t stride, size_t divisor, GLenum _type = GL_FLOAT )
    {
      auto l = get_bind();

      m_array_buffers.push_back( std::make_unique<ArrayBuffer>() );
      auto& new_buffer = m_array_buffers.back();
      new_buffer->put_vector_data( data );
      new_buffer->set_location( loc, dim, stride, _type );
      new_buffer->set_divisor( divisor );
    }

    void bind() override
    {
      glBindVertexArray( m_handle );
        
      if ( m_element_buffer )
      {
        m_element_buffer->bind();
      }

      for ( auto& abuf : m_array_buffers )
      {
        glEnableVertexAttribArray( abuf->location() );
      }
    }

    void unbind() override
    {
      glBindVertexArray( 0 );

      if ( m_element_buffer )
      {
        m_element_buffer->unbind();
      }

      for ( auto& abuf : m_array_buffers )
      {
        glDisableVertexAttribArray( abuf->location() );
      }
    }

  protected:
    std::vector<std::unique_ptr<ArrayBuffer>> m_array_buffers;
    std::unique_ptr<ELementArrayBuffer> m_element_buffer;
};

class Shader : public IGlResource
{
  public:

    ~Shader() = default;

    Shader( GLenum shader_type, const std::string& path )
    {
      m_handle = glCreateShader( shader_type );
      m_source = read_file( path );
    }

    void bind() override
    {
      auto cstr = m_source.c_str();
      glShaderSource( m_handle, GLsizei(1), &cstr, NULL );
      glCompileShader( m_handle );
    }

    void unbind() override
    {
      glDeleteShader( m_handle );
    }

  protected:
    std::string m_source;

};

class ShaderProgram : public IGlResource
{
  public:
    ~ShaderProgram() = default;

    ShaderProgram( const std::string& vertex, const std::string& fragment )
    :  m_vertex( GL_VERTEX_SHADER, vertex ), m_fragment( GL_FRAGMENT_SHADER, fragment )
    {
      m_handle = glCreateProgram();      
    }

    std::string compile()
    {
      m_vertex.bind();
      m_fragment.bind();

      glAttachShader( m_handle, m_vertex.handle() );
      glAttachShader( m_handle, m_fragment.handle() );

      glLinkProgram( m_handle );

      GLsizei log_length = 0;
      glGetProgramiv( m_handle, GL_INFO_LOG_LENGTH, &log_length );

      std::string infolog ( log_length + 1, '\0' );
      glGetProgramInfoLog( m_handle, log_length, NULL, infolog.data() );

      m_compiled = log_length == 0;
      
      return infolog;
    }

    bool valid()
    {
      return m_compiled;
    }

    void bind() override
    {
      glUseProgram( m_handle );
    }

    void unbind() override
    {
      glUseProgram( 0 );
    }

    GLint get_uloc( const std::string& name )
    {
      return glGetUniformLocation( m_handle, name.c_str() );
    }

    void set_uniform_m4f( const std::string& name, const Eigen::Matrix4f& m )
    {
      glUniformMatrix4fv(  get_uloc( name ), 1, GL_FALSE, m.data() );
    }

    void set_uniform_f( const std::string& name, float f )
    {
      glUniform1f( get_uloc( name ), f );
    }

    template<typename T>
    static std::shared_ptr<T> make_program( const std::string& v, const std::string& f )
    {
      auto ptr = std::make_shared<T>( v, f );

      std::cerr << "compilation result:\n" << ptr->compile() << std::endl;
      if ( !ptr->valid() )
      {
        throw std::runtime_error( "compilation fail!" );
      }

      return ptr;
    }
  
  protected:
    Shader m_vertex, m_fragment;
    bool m_compiled = false;

};

class Texture : public IGlResource
{

  public:

    struct Params
    {
      GLenum target = GL_TEXTURE_2D;
      GLint level_of_detail = 0;
      GLint internalformat = GL_RGBA;
      GLsizei width = 752;
      GLsizei height = 480;
      size_t channels = 4;
      GLint border = 0;
      GLenum format = GL_RGBA;
      GLenum type = GL_UNSIGNED_BYTE;
      GLint min_filter = GL_NEAREST;
      GLint mag_filter = GL_NEAREST;
      std::string id = "not specified";
      size_t element_size = sizeof(unsigned char);

      static Params color( GLsizei w, GLsizei h )
      {
        Params p 
        {
          .width = w,
          .height = h,
          .id = "color"
        };

        return p;
      }

      static Params depth( GLsizei w, GLsizei h )
      {
        Params p
        {
          .target = GL_TEXTURE_2D,
          .internalformat = GL_DEPTH_COMPONENT32,
          .width = w,
          .height = h,
          .channels = 1,
          .format = GL_DEPTH_COMPONENT,
          .type = GL_FLOAT,
          .id = "depth",
          .element_size = sizeof(float)
        };

        return p;
      }

      static Params integer( GLsizei w, GLsizei h )
      {
        Params p
        {
          .target = GL_TEXTURE_2D,
          .internalformat = GL_R32I,
          .width = w,
          .height = h,
          .channels = 1,
          .format = GL_RED_INTEGER,
          .type = GL_INT,
          .id = "integer",
          .element_size = sizeof(int32_t)
        };

        return p;
      }

    };


    Texture( const Params& params )
    : m_params( params )
    {
      glGenTextures( 1, &m_handle );

      auto _ = get_bind();

      glTexImage2D(
        m_params.target,
        m_params.level_of_detail,
        m_params.internalformat,
        m_params.width,
        m_params.height,
        m_params.border,
        m_params.format,
        m_params.type,
        NULL
      );

      glTexParameteri( m_params.target, GL_TEXTURE_MIN_FILTER, m_params.min_filter );
      glTexParameteri( m_params.target, GL_TEXTURE_MAG_FILTER, m_params.mag_filter );
    }

    ~Texture()
    {
      glDeleteTextures( 1, &m_handle );
    }

    void bind() override
    {
      glBindTexture( m_params.target, m_handle );
    }

    void unbind() override
    {
      glBindTexture( m_params.target, 0 );
    }

    template<typename T>
    std::vector<T> pull_data()
    {
      std::vector<T> out ( m_params.height * m_params.width * m_params.channels );

      if ( sizeof(T) != m_params.element_size )
      {
        throw std::logic_error( "Trying to access texture with incorrect element type!" );
      }
      
      auto _ = get_bind();

      glGetTexImage(
        m_params.target,
        m_params.level_of_detail,
        m_params.format,
        m_params.type,
        out.data()
      );

      return out;
    }

  protected:
    Params m_params;
};

class Framebuffer : public IGlResource
{
  public:
    
    Framebuffer()
    {
      glGenFramebuffers( 1, &m_handle );
    }

    ~Framebuffer()
    {
      glDeleteFramebuffers( 1, &m_handle );
    }
    
    void create_texture( const Texture::Params& tp, GLenum attachment )
    {
      m_textures[tp.id] = std::make_unique<Texture>( tp );

      auto& new_texture = *m_textures[tp.id];

      auto _ = get_bind();
      glFramebufferTexture2D( GL_FRAMEBUFFER, attachment, tp.target, new_texture.handle(), tp.level_of_detail );

      if ( glCheckFramebufferStatus( GL_FRAMEBUFFER ) != GL_FRAMEBUFFER_COMPLETE )
      {
        throw std::runtime_error( "Framebuffer not complete! Fail on " + tp.id );
      }

      std::cerr << "Created and attached texture " << tp.id << "\n";
    }

    void bind() override
    {
      glBindFramebuffer( GL_FRAMEBUFFER, m_handle );
    }

    void unbind() override
    {
      glBindFramebuffer( GL_FRAMEBUFFER, 0 );
    }

    Texture& get_texture( const std::string& id )
    {
      return *m_textures[id];
    }


  protected:
    std::unordered_map<std::string, std::unique_ptr<Texture>> m_textures;

};

} // mvdb::opengl