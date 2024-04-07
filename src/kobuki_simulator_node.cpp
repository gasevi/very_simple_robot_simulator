#include "kobuki_simulator.h"

#include <thread>
#include <rclcpp/rclcpp.hpp>

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  KobukiSimulator kobuki_simulator( "kobuki_simulator" );
  std::thread main_loop_thread( kobuki_simulator.main_loop );
  rclcpp::spin( kobuki_simulator );
  main_loop_thread.join();
  rclcpp::shutdown();
  return 0;
}

