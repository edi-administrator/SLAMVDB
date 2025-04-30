#include "ros_utils.hxx"
#include "buffer_nodes.hxx"

using namespace mvdb;
using std::placeholders::_1;

class Republisher : public rclcpp::Node
{
  public:
    
    // Republisher() : m_filter( 1.5, -0.75, 1 ), rclcpp::Node( "publisher_node" )
    Republisher() : m_filter( 1.5, 1, -0.75 ), rclcpp::Node( "publisher_node" )
    {

      m_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
        "/points", 
        10, 
        std::bind( &Republisher::sub_cb, this, _1 )
      );

      m_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>( "/filtered_points", 10 );

      RCLCPP_INFO( this->get_logger(), "node up!" );
    };

    void sub_cb( sensor_msgs::msg::PointCloud2::UniquePtr msg )
    {
      RCLCPP_INFO( this->get_logger(), "points received" );

      std::vector<coord_t> coords;

      try
      {
        coords = coord_from_pcd( *msg, m_filter, 4, 1, false );
        // coords = coord_from_pcd( *msg, IdentityConstraint(), 1, 8, false );
      }
      catch(const std::exception& e)
      {
        std::cerr << e.what() << '\n';
      }
      
      m_pub->publish( pcd_from_coord( coords, "os_sensor" ) );
    }

  
  protected:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub;
    PointCropConstriant m_filter;

};

int main( int argc, char** argv )
{
  
  rclcpp::init( argc, argv );
  auto nh = std::make_shared<Republisher>();

  while ( rclcpp::ok() )
  {
    rclcpp::spin( nh );
  }

  rclcpp::shutdown();
}