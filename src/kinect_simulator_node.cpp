#include "kinect_simulator.h"

#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<KinectSimulator>( "kinect_simulator" ) );
  rclcpp::shutdown();
  return 0;
}

