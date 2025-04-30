
#include "projection.hxx"
#include "config.hxx"

#ifdef __USE_OPENGL
#include "gpu_projector.hpp"
#endif

using namespace mvdb;

std::shared_ptr<IProjector> make_projector( const ConfigReader& cfr )
{
  #ifdef __USE_OPENGL
  #pragma message( "Compiling WITH OpenGL support!" )
    std::cerr << "Creating GPU projector!\n";
    auto projparam = cfr.get_projection_defaults();
    static auto projector = std::make_shared<opengl::OpenGLThreadWrapper>( projparam );
    return projector;
  #else
  #pragma message( "Compiling WITHOUT OpenGL support!" )
    std::cerr << "Creating CPU projector!\n";
    return std::make_shared<CPUProjectorWrapper>( cfr.get_projection_defaults() );
  #endif
}

std::vector<coord_t> make_test()
{
  std::vector<coord_t> out;
  for ( double z = -10; z < 10; z += 0.5 )
  {
    for ( double y = -10; y < 10; y += 0.5 )
    {
      out.push_back( coord_t { 0, y, z} );
    }
  }
  return out;
}

int main()
{
  
  using sem_t = mvdb::sem_t;

  ConfigReader cfr {};
  auto renderer1 = make_projector( cfr );
  auto renderer2 = make_projector( cfr );

  std::cout << "main thread active" << std::endl;
  std::cout << "renderer1 at " << renderer1 << std::endl;
  std::cout << "renderer2 at " << renderer2 << std::endl;

  Eigen::Isometry3d cpose = Eigen::Isometry3d::Identity();
  cpose =  cpose.pretranslate( Eigen::Vector3d { -15, 0, 0 } );

  std::vector<coord_t> positions 
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

  std::vector<mvdb::sem_t> image;
  for ( size_t i = 0; i < 192 * 128; i++ )
  {
    image.push_back( sem_t::Zero() );
  }


  RenderingRequest r
  {
    .points = positions,
    .camera_poses = { cpose.matrix() },
    .images = { std::make_shared<std::vector<sem_t>>( image ) }
  };

  std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
  renderer1->render( std::make_shared<RenderingRequest>( r ) );

}