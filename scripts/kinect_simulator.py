#!/usr/bin/env python3
  
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from tf_transformations import euler_from_quaternion
import numpy as np
import cv2
from cv_bridge import CvBridge

from utils import CoordinateConverter
from rangefinder import build_pixel_rangefinder


class KinectSimulator( Node ):

  def __init__( self ):
    super().__init__( 'kinect_simulator' )
    self.kinect_height = 0.3 # [m]
    self.wall_height = 0.5 # [m]
    self.max_depth = 10.0 # [m]
    self.hfov = 57*np.pi/180.0 # [rad] (57 [degrees])
    self.vfov = 43*np.pi/180.0 # [rad] (43 [degrees])
    self.view_depth = 4.0 # [m]
    self.min_valid_distance = 0.45 # [m]
    self.depth_img_width = 640 # [pix]
    self.depth_img_height = 480 # [pix]
    self.map_resolution = 0.01 # [m/pix]

    self.n_h_scans = 50
    self.n_v_scans = int( (self.depth_img_height * self.n_h_scans) / self.depth_img_width )
    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]
    self.h_beam_angles = np.linspace( self.hfov/2.0, -self.hfov/2.0, self.n_h_scans )
    self.v_beam_angles = np.linspace( self.vfov/2.0, -self.vfov/2.0, self.n_v_scans )

    self.converter = None
    self.mapimg = np.array( [] )
    self.cv_bridge = CvBridge()
    #rospy.Subscriber( '/real_pose', Pose, self.new_pose, queue_size = 1 )
    self.create_subscription( Pose, '/real_pose', self.new_pose, 1 )
    #rospy.Subscriber( 'map', OccupancyGrid, self.set_map )
    self.create_subscription( OccupancyGrid, 'map', self.set_map, 1 )
    #self.pub_depth = rospy.Publisher( 'camera/depth/image_raw', Image, queue_size = 1 )
    self.pub_depth = self.create_publisher( Image, 'camera/depth/image_raw', 1 )

  def new_pose( self, pose ):
    if len( self.mapimg ) == 0:
      return None

    depth_image = self.max_depth * np.ones( ( self.n_v_scans, self.n_h_scans ), dtype = np.float32 )
    x, y = self.converter.metric2pixel( pose.position.x, pose.position.y )
    if y < 0 or self.mapimg.shape[0] <= y or x < 0 or self.mapimg.shape[1] <= x:
      depth_image = cv2.resize( depth_image, ( self.depth_img_width, self.depth_img_height ) )
      msg = self.cv_bridge.cv2_to_imgmsg( depth_image )
      self.pub_depth.publish( msg )
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
    distance_sensor = distance_sensor[::-1] # reverse: from left to right
    for c, d in enumerate( distance_sensor ):
      d = d*np.cos( self.h_beam_angles[c] ) # project beam into the robot plane

      ceiling_angle = np.arctan2( self.wall_height - self.kinect_height, d )
      ceiling_limit_index = -1
      if ceiling_angle < self.vfov/2.0:
        ceiling_indices = np.where( self.v_beam_angles > ceiling_angle )[0]
        depth_image[ceiling_indices,c] = self.max_depth
        ceiling_limit_index = ceiling_indices.max()

      ground_angle = np.arctan2( self.kinect_height, d )
      ground_limit_index = self.n_v_scans
      if ground_angle < self.vfov/2.0:
        ground_indices = np.where( self.v_beam_angles < -ground_angle )[0]
        for i in ground_indices:
          ground_d = self.kinect_height / np.sin( abs( self.v_beam_angles[i] ) )
          depth_image[i,c] = ground_d if ground_d >= self.min_valid_distance else float( 'nan' )
        ground_limit_index = ground_indices.min()

      if ceiling_limit_index < 0:
        depth_image[0:ground_limit_index,c] = d if d >= self.min_valid_distance else float( 'nan' )
      else:
        depth_image[ceiling_limit_index+1:ground_limit_index,c] = d if d >= self.min_valid_distance else float( 'nan' )
    depth_image = cv2.resize( depth_image, ( self.depth_img_width, self.depth_img_height ) )

    msg = self.cv_bridge.cv2_to_imgmsg( depth_image ) #, encoding = '32FC1' )
    self.pub_depth.publish( msg )

  def set_map( self, occupancy_grid ):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    origin_x = occupancy_grid.info.origin.position.x
    origin_y = occupancy_grid.info.origin.position.y
    self.map_resolution = occupancy_grid.info.resolution
    self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
    self.mapimg = np.flip( self.mapimg, axis = 0 )
    self.converter = CoordinateConverter( origin_x, origin_y, self.map_resolution, height )
    self.view_depth_pix = self.view_depth / self.map_resolution # [pix]


if __name__ == '__main__':
  rclpy.init()
  kinect_simulator = KinectSimulator()
  rclpy.spin( kinect_simulator )
  rclpy.shutdown()

