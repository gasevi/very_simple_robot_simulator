#include "kobuki_simulator.h"

#include <ros/ros.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "kobuki_simulator" );
  KobukiSimulator kobuki_simulator;
  kobuki_simulator.main_loop();
  return 0;
}

