#!/usr/bin/env python

# Dependencies:
#   apt install python-imaging-tk
  
import os.path
import signal
import rospy
from geometry_msgs.msg import Pose, Quaternion, Point
from std_msgs.msg import Header
from nav_msgs.msg import MapMetaData, OccupancyGrid
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from PIL import Image as PILImage, ImageDraw as PILImageDraw, ImageTk
from Tkinter import *
import tkFileDialog
import cv2
from cv_bridge import CvBridge
import yaml

from utils import CoordinateConverter

class WorldStateGUI( Frame ):

  def __init__( self, width = 917, height = 483, resolution = 0.006 ):
    self.width = width
    self.height = height
    self.resolution = resolution # [m/pix]
    self.robot_diameter = 0.355 # [m]
    self.robot_radio = self.robot_diameter / 2.0 # [m]
    self.robot_radio_pix = int( self.robot_radio / self.resolution )
    self.converter = CoordinateConverter( 0.0, height * self.resolution, self.resolution )
    self.cv_bridge = CvBridge()

    self.root = Tk()
    self.root.geometry( '%dx%d' % (width, height) )
    self.root.title( 'World State' )
    self.root.resizable( False, False )
    Frame.__init__( self, self.root, width = width, height = height )
    self.grid( row = 0, column = 0 )

    menubar = Menu( self.master )
    self.master.config( menu = menubar )
    fileMenu = Menu( menubar )
    fileMenu.add_command( label = "Open map ...", command = self.open_map )
    fileMenu.add_command( label = "Exit", command = self.on_exit )
    menubar.add_cascade( label = "File", menu = fileMenu )

    self.canvas = Canvas( self, width = width, height = height, bg = '#FFFFFF' )
    self.canvas.pack( side = LEFT, expand = True, fill = BOTH )
    self.canvas.pilimage = None
    self.canvas.bgimage = None
    self.canvas.configure( cursor = 'left_ptr' )
    self.canvas.focus_set()

    signal.signal( signal.SIGINT, self.sigint_handler )

    rospy.Subscriber( 'real_pose', Pose, self.refresh )
    self.pub_map_metadata = rospy.Publisher( 'map_metadata', MapMetaData, queue_size = 1, latch = True )
    self.pub_map = rospy.Publisher( 'map', OccupancyGrid, queue_size = 1, latch = True )

    map_quat = quaternion_from_euler( 0.0, 0.0, 0.0 )
    map_pose = Pose( Point( 0.0, height * self.resolution, 0.0 ), Quaternion( *map_quat ) )
    map_metadata = MapMetaData( rospy.Time.now(), resolution, width, height, map_pose )
    self.pub_map_metadata.publish( map_metadata )

    og_header = Header( 0, rospy.Time.now(), 'map_frame' )
    og_data = np.zeros( height * width, dtype = np.uint8 )
    occupancy_grid = OccupancyGrid( og_header, map_metadata, og_data.tolist() )
    self.pub_map.publish( occupancy_grid )

  def open_map( self ):
    yamlfile = tkFileDialog.askopenfilename( title = 'Load Map', filetypes = [ ( 'YAML', ( '*.yaml' ) ) ] )
    if len( yamlfile ) == 0:
      return

    with open( yamlfile ) as fd:
      metadata = yaml.load( fd )

    if not os.path.isabs( metadata['image'] ):
      map_path = os.path.dirname( yamlfile )
      map_filename = os.path.basename( metadata['image'] )
      map_file = os.path.join( map_path, map_filename )

    self.resolution = metadata['resolution'] # [m/pix]
    self.robot_radio_pix = int( self.robot_radio / self.resolution )
    self.converter = CoordinateConverter( metadata['origin'][0], metadata['origin'][1], self.resolution )

    pilimage = PILImage.open( map_file )
    width, height = pilimage.size
    bgimage = ImageTk.PhotoImage( pilimage )

    # keep a reference to the image to avoid the image being garbage collected
    self.canvas.pilimage = pilimage
    self.canvas.bgimage = bgimage
    self.canvas.bgimagefile = map_file # TO-DO: remove this and just use bgimage to save screen
    self.canvas.delete( 'backgroundimg' )
    self.canvas.create_image( 0, 0, anchor = NW, image = bgimage, tags = 'backgroundimg' )
    self.root.geometry( '%dx%d' % (width, height) )

    width, height = pilimage.size

    map_quat = quaternion_from_euler( 0.0, 0.0, 0.0 )
    map_pose = Pose( Point( 0.0, height * self.resolution, 0.0 ), Quaternion( *map_quat ) )
    map_metadata = MapMetaData( rospy.Time.now(), self.resolution, width, height, map_pose )
    self.pub_map_metadata.publish( map_metadata )

    og_header = Header( 0, rospy.Time.now(), 'map_frame' )
    og_data = np.array( pilimage )
    if len( og_data.shape ) > 2:
      og_data = cv2.cvtColor( og_data, cv2.COLOR_BGR2GRAY )
    og_data = ( 100 - ( og_data / 255.0 ) * 100 ).astype( np.uint8 )
    og_data = og_data.reshape( height * width )
    occupancy_grid = OccupancyGrid( og_header, map_metadata, og_data.tolist() )
    self.pub_map.publish( occupancy_grid )

  def refresh( self, pose ):
    x, y = self.converter.cartesian2pixel( pose.position.x, pose.position.y )
    roll, pitch, yaw = euler_from_quaternion( ( pose.orientation.x,
                                                pose.orientation.y,
                                                pose.orientation.z,
                                                pose.orientation.w ) )
    self.canvas.delete( 'robot' )
    self.canvas.delete( 'robot_direction' )
    self.canvas.create_oval( x-self.robot_radio_pix,
                             y-self.robot_radio_pix,
                             x+self.robot_radio_pix,
                             y+self.robot_radio_pix,
                             outline = 'red',
                             fill = '',
                             tags = 'robot' )
    x1 = int( x + self.robot_radio_pix * np.cos( yaw ) )
    y1 = int( y - self.robot_radio_pix * np.sin( yaw ) )
    self.canvas.create_line( x, y, x1, y1, fill = 'red', tags = 'robot_direction' )
    #rospy.loginfo( 'x: %d, y: %d, yaw: %f' % ( x, y, yaw ) )

  def mainloop( self ):
    self.root.mainloop()

  def sigint_handler( self, sig, frame ):
    self.root.quit()
    self.root.update()

  def on_exit( self ):
    self.quit()


if __name__ == '__main__':
  rospy.init_node( 'world_state_gui' )
  world_state_gui = WorldStateGUI()
  world_state_gui.mainloop()


