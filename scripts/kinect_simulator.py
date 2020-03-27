#!/usr/bin/env python
  
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from cv_bridge import CvBridge

from utils import CoordinateConverter

def sawtooth( x ):
   return (x-np.pi) % (2*np.pi) - np.pi

def build_pixel_beam( global_map, robot_pose, max_len = 50.0 ):
  x0, y0, angle = robot_pose
  y0 = -y0
  angle = sawtooth( angle )
  if len( global_map.shape ) > 2:
    h, w, _ = global_map.shape
  else:
    h, w = global_map.shape
  if abs( np.tan( angle ) ) > 10**4:
    if angle > 0:
      dy = 0-y0
      dx = 0
    else:
      dy = (-h)-y0
      dx = 0
  else:
    a = np.tan( angle )
    b = y0-a*x0
    xp1 = -(b/a)
    xp2 = -(h+b)/a
    yp1 = a*w+b
    yp2 = b
    if 0 <= angle and angle <= np.pi:
      if xp1 <= 0:
        dy = int( round( yp2-y0 ) )
        dx = 0-x0
      elif 0 < xp1 and xp1 <= w:
        dy = 0-y0
        dx = int( round( xp1-x0 ) )
      else:
        dy = int( round( yp1-y0 ) )
        dx = w-x0
    elif -np.pi <= angle and angle < 0:
      if xp2 <= 0:
        dy = int( round( yp2-y0 ) )
        dx = 0-x0
      elif 0 < xp2 and xp2 <= w:
        dy = (-h)-y0
        dx = int( round( xp2-x0 ) )
      else:
        dy = int( round( yp1-y0 ) )
        dx = w-x0
  steps = abs(dx) if abs(dx) > abs(dy) else abs(dy)
  x_inc = dx / float( steps )
  y_inc = dy / float( steps )
  beam_len = np.hypot( dx, dy )
  f = max_len/beam_len if beam_len > max_len else 1
  trimmed_steps = int( round( f*steps ) )
  x = x0
  y = y0
  pixel_beam = list()
  for i in range( trimmed_steps ):
    if np.all( global_map[-int(y)][int(x)] == 0 ):
      break
    pixel_beam.append( [-int(y), int(x)] )
    x += x_inc
    y += y_inc
    if x >= w or y >= h:
      break
  return pixel_beam

def build_pixel_lidar( global_map, pose, fov, n_scans = 100, view_depth = 60 ):
  x, y, yaw = pose
  left_beam = yaw + fov/2.0
  right_beam = yaw - fov/2.0
  pixel_lidar = list()
  distance_sensor = list()
  for angle in np.linspace( left_beam, right_beam, n_scans ).tolist():
    robot_pose = (x, y, angle)
    pixel_beam = build_pixel_beam( global_map, robot_pose, view_depth )
    pixel_lidar.append( pixel_beam )
    d = np.sqrt( (pixel_beam[-1][0] - pixel_beam[0][0])**2 + (pixel_beam[-1][1] - pixel_beam[0][1])**2 )
    distance_sensor.append( d )
  return pixel_lidar, distance_sensor

def lidar_drawer( canvas, robot_pose, pixel_lidar ):
  for beam in pixel_lidar:
    for y, x in beam:
      canvas[y][x][0] = 0
      canvas[y][x][1] = 0
      canvas[y][x][2] = 255
  x, y, yaw = robot_pose
  canvas[y][x][0] = 255
  canvas[y][x][1] = 0
  canvas[y][x][2] = 0

class KinectSimulator( object ):

  def __init__( self ):
    self.kinect_height = 0.3 # [m]
    self.wall_height = 0.5 # [m]
    self.max_depth = 10.0 # [m]
    self.hfov = 57*np.pi/180.0 # [rad] (57 [degrees])
    self.vfov = 43*np.pi/180.0 # [rad] (43 [degrees])
    self.depth_img_width = 640 # [pix]
    self.depth_img_height = 480 # [pix]
    self.map_resolution = 0.01 # [m/pix]

    self.show_depth_map = False
    self.n_h_scans = 50
    self.n_v_scans = int( (self.depth_img_height * self.n_h_scans) / self.depth_img_width )
    self.view_depth_pix = 4.0 / self.map_resolution # [pix]
    self.h_beam_angles = np.linspace( self.hfov/2.0, -self.hfov/2.0, self.n_h_scans )
    self.v_beam_angles = np.linspace( self.vfov/2.0, -self.vfov/2.0, self.n_v_scans )

    self.converter = None
    self.mapimg = np.array( [] )
    self.cv_bridge = CvBridge()
    rospy.Subscriber( '/real_pose', Pose, self.new_pose )
    rospy.Subscriber( 'map', OccupancyGrid, self.set_map )
    self.pub_depth = rospy.Publisher( 'camera/depth/image_raw', Image, queue_size = 10 )

  def new_pose( self, pose ):
    if len( self.mapimg ) == 0:
      return None
    depth_image = self.max_depth * np.ones( ( self.n_v_scans, self.n_h_scans ), dtype = np.float32 )
    x, y = self.converter.cartesian2pixel( pose.position.x, pose.position.y )
    roll, pitch, yaw = euler_from_quaternion( ( pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w ) )

    robot_pose = (x, y, yaw)
    pixel_lidar, distance_sensor = build_pixel_lidar( self.mapimg, robot_pose, self.hfov, self.n_h_scans, self.view_depth_pix )
    distance_sensor = self.map_resolution * np.array( distance_sensor ) # [m]
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
          depth_image[i,c] = self.kinect_height / np.sin( abs( self.v_beam_angles[i] ) )
        ground_limit_index = ground_indices.min()

      if ceiling_limit_index < 0:
        depth_image[0:ground_limit_index,c] = d
      else:
        depth_image[ceiling_limit_index+1:ground_limit_index,c] = d
    depth_image = cv2.resize( depth_image, ( self.depth_img_width, self.depth_img_height ) )

    if self.show_depth_map:
      image2display = depth_image - depth_image.min()
      image2display = ( image2display * ( 255.0/image2display.max() ) ).astype( np.uint8 )
      image2display = 255 - image2display
      image2display = cv2.applyColorMap( image2display, cv2.COLORMAP_HOT )
      cv2.imshow( 'Depth Sensor', image2display )
      cv2.waitKey( 1 )

    msg = self.cv_bridge.cv2_to_imgmsg( depth_image ) #, encoding = '32FC1' )
    self.pub_depth.publish( msg )

  def set_map( self, occupancy_grid ):
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    self.map_resolution = occupancy_grid.info.resolution
    self.mapimg = 100 - np.array( occupancy_grid.data ).reshape( (height, width) )
    self.converter = CoordinateConverter( 0.0, self.mapimg.shape[0] * self.map_resolution, self.map_resolution )


if __name__ == '__main__':
  rospy.init_node( 'kinect_simulator' )
  kinect_simulator = KinectSimulator()
  rospy.spin()

