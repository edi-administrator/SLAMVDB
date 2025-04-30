#include "buffer_nodes.hxx"
#include "mapper_nodes.hxx"
#include "utils.hxx"
#include "config.hxx"

#ifdef __USE_OPENGL
#include "gpu_projector.hpp"
#endif

using namespace mvdb;

std::shared_ptr<IProjector> make_projector( const ConfigReader& cfr )
{
  #ifdef __USE_OPENGL
  #pragma message( "Compiling main WITH OpenGL " )
    std::cerr << "Creating GPU projector!\n";
    auto projparam = cfr.get_projection_defaults();
    static auto projector = std::make_shared<opengl::OpenGLThreadWrapper>( projparam );
    return projector;
  #else
  #pragma message( "Compiling main WITHOUT OpenGL " )
    std::cerr << "Creating CPU projector!\n";
    return std::make_shared<CPUProjectorWrapper>( cfr.get_projection_defaults() );
  #endif
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  
  ConfigReader cfr {};

  auto nhi = std::make_shared<ImageSubBuffer>();
  auto nhs = std::make_shared<ScanBuffer>( cfr.get_scan_buffer_defaults() );
  auto nhp = std::make_shared<PoseBuffer>( cfr.get_pose_buffer_params( "tracker" ) );
  auto nhmp = std::make_shared<PoseBuffer>( cfr.get_pose_buffer_params( "mapper" ) );

  exec.add_node(nhi);
  exec.add_node(nhs);
  exec.add_node(nhp);
  exec.add_node(nhmp);

  auto quantizer = std::make_shared<Quantizer>( cfr.get_quantizer_defaults() );

  auto global_mapper_params = cfr.get_global_mapper_defaults();
  global_mapper_params.global_vox_params.quantizer = quantizer;
  global_mapper_params.pbuf = nhmp;
  global_mapper_params.tbuf = nhp;
  global_mapper_params.projector = make_projector( cfr );
  auto nhgm = std::make_shared<GlobalMapper>( global_mapper_params );
  exec.add_node(nhgm);


  auto local_mapper_params = cfr.get_local_mapper_defaults();
  local_mapper_params.quantizer = quantizer;
  local_mapper_params.ibuf = nhi;
  local_mapper_params.pbuf = nhp;
  local_mapper_params.sbuf = nhs;
  local_mapper_params.gmap = nhgm;
  local_mapper_params.projector = make_projector( cfr );
  auto nhlm = std::make_shared<LocalMapper>( local_mapper_params );
  exec.add_node(nhlm);

  std::cerr << "reached spin stage" << std::endl;
  while ( rclcpp::ok() )
  {
    exec.spin();
  }
  rclcpp::shutdown();

}