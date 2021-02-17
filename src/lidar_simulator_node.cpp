#include "lidar_simulator.h"

#include <ros/ros.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "lidar_simulator" );
  LidarSimulator lidar_simulator( true );
  ros::spin();
  return 0;
}

