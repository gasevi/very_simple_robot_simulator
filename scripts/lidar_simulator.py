#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import numpy as np

from rangefinder import build_pixel_rangefinder
from utils import CoordinateConverter


class LidarSimulator( object ):

  def __init__( self ):
    self.hfov = 57*np.pi/180.0 # [rad] (57 [degrees])
    self.n_h_scans = 57
    self.map_resolution = 0.01 # [m/pix]
    self.view_depth = 4.0 # [m]
    self.z_max = self.view_depth # [m]
    self.std_error = 0.02 # [m]

    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]
    self.seq = 0
    self.converter = None
    self.mapimg = np.array( [] )
    self.h_beam_angles = np.linspace( self.hfov/2.0, -self.hfov/2.0, self.n_h_scans )

    self.lidar_fov = np.pi
    self.lidar_n_h_scans = 181

    rospy.Subscriber( 'map', OccupancyGrid, self.set_map )
    rospy.Subscriber( '/real_pose', Pose, self.new_pose, queue_size = 1 )
    self.scan_pub = rospy.Publisher( '/scan', LaserScan, queue_size = 10 )

  def new_pose( self, pose ):
    if len( self.mapimg ) == 0:
      return None

    x, y = self.converter.metric2pixel( pose.position.x, pose.position.y )
    if y < 0 or self.mapimg.shape[0] <= y or x < 0 or self.mapimg.shape[1] <= x:
      scans = self.z_max * np.ones( self.lidar_n_h_scans )
      self.send_laser_scan( scans )
      return None

    roll, pitch, yaw = euler_from_quaternion( ( pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w ) )

    robot_pose = (x, y, yaw)
    pixel_lidar, distance_sensor = build_pixel_rangefinder( self.mapimg,
                                                            robot_pose,
                                                            self.hfov,
                                                            self.n_h_scans,
                                                            self.view_depth_pix )
    distance_sensor = self.map_resolution * np.array( distance_sensor ) # [m]
    distance_sensor = np.random.normal( distance_sensor, self.std_error )
    distance_sensor[distance_sensor > self.z_max-0.2] = self.z_max # filter pix to meter conversion errors

    scans = np.zeros( self.lidar_n_h_scans )
    out_of_fov_beams = int( self.lidar_n_h_scans/2 ) - int( self.n_h_scans/2 )
    scans[:out_of_fov_beams] = self.z_max
    scans[-out_of_fov_beams:] = self.z_max
    scans[out_of_fov_beams:-out_of_fov_beams] = distance_sensor

    self.send_laser_scan( scans )

  def send_laser_scan( self, scans ):
    self.seq += 1
    laserScan = LaserScan()
    laserScan.header.seq = self.seq
    laserScan.header.stamp = rospy.Time.now()
    laserScan.header.frame_id = "base_link"
    laserScan.angle_min = -self.lidar_fov/2.0
    laserScan.angle_max = self.lidar_fov/2.0
    laserScan.angle_increment = 1.0*np.pi/180.0
    laserScan.time_increment = 0.00001
    laserScan.scan_time = 0.001*181
    laserScan.range_min = 0.0
    laserScan.range_max = self.z_max
    laserScan.ranges = scans
    laserScan.intensities = []
    self.scan_pub.publish( laserScan )

  def set_map( self, occupancy_grid ):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    self.map_resolution = occupancy_grid.info.resolution
    self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
    self.converter = CoordinateConverter( 0.0, self.mapimg.shape[0] * self.map_resolution, self.map_resolution )
    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]


if __name__ == '__main__':
  rospy.init_node( 'lidar_simulator' )
  lidar_sim = LidarSimulator()
  rospy.spin()

