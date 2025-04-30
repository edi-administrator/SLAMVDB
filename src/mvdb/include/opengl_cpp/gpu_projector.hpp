#pragma once

#include "projection.hxx"
#include "detail.hpp"
#include "model.hpp"
#include "renderer.hpp"

#include <future>
#include <queue>
#include <condition_variable>
#include <thread>

namespace mvdb::opengl
{

class GPUProjector
{
  public:

    ~GPUProjector()
    {
      m_program->unbind();
      m_fbo->unbind();
    }

    GPUProjector( const GlfwStateWrapper& glstate, const ProjectionParams& params = ProjectionParams {} )
    : m_params( params ), m_glstate( glstate )
    {
      auto cparams = CameraParams::from_proj_params( params );
      Eigen::Matrix4f projmatrix = intrinsic_to_projection( cparams, params.near, params.far );
      std::cerr << "projmatrix =\n" << projmatrix << "\n";

      m_program = ProgramProjectIndex::make_program();
      m_program->bind();
      m_program->set_constant_uniforms( cparams, projmatrix );

      m_fbo = std::make_shared<Framebuffer>();
      m_fbo->create_texture( Texture::Params::depth( cparams.w, cparams.h ), GL_DEPTH_ATTACHMENT );
      m_fbo->create_texture( Texture::Params::integer( cparams.w, cparams.h ), GL_COLOR_ATTACHMENT0 );
      m_fbo->bind();

      m_T_cv_opengl = pose_t::Identity();
      m_T_cv_opengl.block<3,3>(0,0) = m_T_cv_opengl.block<3,3>(0,0) * Eigen::AngleAxis<double>( k_pi, vec_t::UnitX() ).matrix();
    }

    void project( 
      const OctreeParams& params, 
      const std::vector<coord_t>& points,
      const std::vector<pose_t>& poses, 
      const std::vector<std::shared_ptr<std::vector<sem_t>>>& images, 
      ProjectionResult& result )
    {

      ProgramProjectDistort::VoxelParams vparams
      {
        .scale = float( params.res * 0.8 ),
        .centered = false,
      };
      
      std::vector<xkey_t> keys;
      std::vector<Eigen::Vector3f> positions;
      std::vector<int32_t> indices;

      int32_t index = 1;
      for ( auto& p : points )
      {
        positions.push_back( vec_from_coord( p ).cast<float>() );
        keys.push_back( params.coord_to_xkey_leaf( p ) );
        indices.push_back( index );
        index++;
      }

      std::shared_ptr<InstancedModelGeneric> model;
      if ( positions.size() > 0 )
      {
        model = m_program->make_model( positions, indices, vparams );
      }

      if ( !model )
      {
        return;
      }

      for ( size_t image = 0; image < images.size(); image++ )
      {
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        m_program->set_view_matrix( ( poses[image] * m_params.T_scan_camera * m_T_cv_opengl ).inverse().cast<float>() );
  
        model->draw();
        
        auto projected_indices = m_fbo->get_texture( "integer" ).pull_data<int32_t>();
  
        if ( projected_indices.size() != images[image]->size() * m_params.supersample * m_params.supersample )
        {
          throw std::out_of_range( "projected_indices.size() = " + std::to_string( projected_indices.size() ) + " != " + std::to_string( images[image]->size() ) );
        }
        
        size_t w_super = m_params.w * m_params.supersample;
  
        for ( size_t i = 0; i < projected_indices.size(); i++ )
        {
          auto& p_idx = projected_indices[i];
          if ( p_idx != 0 )
          {
            size_t row_super = i / w_super;
            size_t col_super = i % w_super;
  
            size_t row_sub = row_super / m_params.supersample;
            size_t col_sub = col_super / m_params.supersample;
  
            size_t index_sub = row_sub * m_params.w + col_sub;
  
            result.sem.push_back( images[image]->at(index_sub) );
            result.keys.push_back( keys[p_idx - 1] );
            result.counts.push_back(1);
            result.times.push_back(-1);
          }
        }
      }


    }

  protected:

    pose_t m_T_cv_opengl;
    ProjectionParams m_params;
    const GlfwStateWrapper& m_glstate;

    std::shared_ptr<ProgramProjectIndex> m_program;
    std::shared_ptr<InstancedModelGeneric> m_model;
    std::shared_ptr<Framebuffer> m_fbo;
};


struct RenderingTask
{
  std::shared_ptr<const RenderingRequest> request;
  std::shared_ptr<std::promise<ProjectionResult>> projection_promise;
};


class OpenGLThreadWrapper : public IProjector
{
  public:

    struct Params
    {
      GlfwStateWrapper::Params glfw_params;
      ProjectionParams projection_params;
    };

    OpenGLThreadWrapper( const ProjectionParams& params )
    {
      m_rendering_thread = std::thread( &OpenGLThreadWrapper::worker_thread, this, params );
    }

    ~OpenGLThreadWrapper()
    {
      {
        std::lock_guard<std::mutex> lock_queue ( m_mutex_queue );
        m_stop = true;
      }
      m_condition.notify_one();
      m_rendering_thread.join();
    }

    void worker_thread( const ProjectionParams& params )
    {

      GlfwStateWrapper::Params glfw_params
      {
        .w = params.w * params.supersample,
        .h = params.h * params.supersample,
        .should_show = false
      };

      auto glfw_state = std::make_unique<GlfwStateWrapper>( glfw_params );
      auto projector = std::make_unique<GPUProjector>( *glfw_state, params );

      std::cerr << "worker thread started!\n";

      bool should_exit = false;

      while( true )
      {
        std::vector<std::shared_ptr<const RenderingRequest>> requests;
        std::vector<std::shared_ptr<std::promise<ProjectionResult>>> promises;

        {
          std::unique_lock<std::mutex> condition_lock ( m_mutex_queue );
          m_condition.wait( condition_lock, [&]()->bool{ return ( !m_queue.empty() || m_stop ); } );

          while ( !m_queue.empty() )
          {
            requests.push_back( m_queue.front().request );
            promises.push_back( m_queue.front().projection_promise );
            m_queue.pop();
          }

          should_exit = m_stop;
        }
        
        for ( size_t i = 0; i < requests.size(); i++ )
        {
          auto& req = requests[i];
          auto& promise = promises[i];
          ProjectionResult result;
          if ( !should_exit )
          {
            projector->project( req->tree_params, req->points, req->camera_poses, req->images, result );
          }
          promise->set_value( std::move( result ) );
        }

        if ( should_exit )
        {
          break;
        }
      }
    }

    virtual ProjectionResult render( const std::shared_ptr<const RenderingRequest> request ) override
    {
      RenderingTask task
      {
        .request = request,
        .projection_promise = std::make_shared<std::promise<ProjectionResult>>()
      };

      auto future = task.projection_promise->get_future();

      {
        std::lock_guard<std::mutex> lock ( m_mutex_queue );
        m_queue.push( task );
      }

      m_condition.notify_one();
      future.wait_for( std::chrono::milliseconds(5000) );

      if ( future.valid() )
      {
        return future.get();
      }
      else
      {
        std::cerr << "future timed out!\n";
        return ProjectionResult {}; 
      }

    }


  protected:

    Params m_params;

    std::queue<RenderingTask> m_queue;
    std::mutex m_mutex_queue;
    std::thread m_rendering_thread;
    std::condition_variable m_condition;

    bool m_stop = false;
};

}