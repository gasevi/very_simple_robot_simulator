#include "kinect_simulator.h"

#include <ros/ros.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "kinect_simulator" );
  KinectSimulator kinect_simulator;
  ros::spin();
  return 0;
}

