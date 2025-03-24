#include "buffer_nodes.hxx"
#include "mapper_nodes.hxx"
#include "utils.hxx"
#include "config.hxx"

using namespace mvdb;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  
  ConfigReader cfr {};

  auto nhi = std::make_shared<ImageBuffer>();
  auto nhs = std::make_shared<ScanBuffer>();
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
  auto nhgm = std::make_shared<GlobalMapper>( global_mapper_params );
  exec.add_node(nhgm);

  auto local_mapper_params = cfr.get_local_mapper_defaults();
  local_mapper_params.quantizer = quantizer;
  local_mapper_params.ibuf = nhi;
  local_mapper_params.pbuf = nhp;
  local_mapper_params.sbuf = nhs;
  local_mapper_params.gmap = nhgm;
  auto nhlm = std::make_shared<LocalMapper>( local_mapper_params );
  exec.add_node(nhlm);

  rclcpp::on_shutdown(
    [&]()
    {
      std::cout << "shutdown triggered!" << "\n";
      std::cout << "nhlm " << nhlm << "\n";
      std::cout << "nhlm map size " << nhlm->map().filter_xkeys( IdentityConstraint() ).size() << "\n";
    }
  );

  while ( rclcpp::ok() )
  {
    exec.spin();
  }
  rclcpp::shutdown();

}