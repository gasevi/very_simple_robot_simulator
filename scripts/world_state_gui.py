#!/usr/bin/env python

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


class CanvasMode( object ):

  def __init__( self, canvas ):
    pass

  def click1( self, event ):
    pass

  def click1_off( self, event ):
    pass

  def click1_motion( self, event ):
    pass

  def double_click1( self, event ):
    pass


class IdleMode( CanvasMode ):
  pass


class SetRobotLocationMode( CanvasMode ):

  def __init__( self, canvas, publisher ):
    self.canvas = canvas
    self.publisher = publisher

  def click1( self, event ):
    x = self.canvas.canvasx( event.x )
    y = self.canvas.canvasy( event.y )
    quat = quaternion_from_euler( 0.0, 0.0, 0.0 )
    pix_pose = Pose( Point( x, y, 0.0 ), Quaternion( *quat ) )
    self.publisher.publish( pix_pose )


class AddWallMode( CanvasMode ):

  def __init__( self, canvas, id_offset = 0 ):
    self.canvas = canvas
    self.x = 0
    self.y = 0
    self.id_offset = id_offset
    self.current_tag = ''

  def id_offset( self, id_offset ):
    self.id_offset = id_offset

  def click1( self, event ):
    self.x = self.canvas.canvasx( event.x )
    self.y = self.canvas.canvasy( event.y )
    self.current_tag = 'wall_' + str( self.id_offset )
    self.canvas.create_line( self.x, self.y, self.x, self.y, width = 3, fill = 'black', tags = self.current_tag )

  def click1_off( self, event ):
    self.id_offset += 1

  def click1_motion( self, event ):
    canvasx = self.canvas.canvasx( event.x )
    canvasy = self.canvas.canvasy( event.y )
    deltax = canvasx - self.x
    deltay = canvasy - self.y
    coords = self.canvas.coords( self.current_tag )
    coords[2] += deltax
    coords[3] += deltay
    self.canvas.coords( self.current_tag, *coords )
    self.x = canvasx
    self.y = canvasy

  def reset( self ):
    self.id_offset = 0


class DeleteWallMode( CanvasMode ):

  def __init__( self, canvas ):
    self.canvas = canvas

  def click1( self, event ):
    current_tag = self.canvas.itemcget( CURRENT, 'tags' ).split( ' ' )[0]
    self.canvas.delete( current_tag )


class WorldStateGUI( Frame ):

  def __init__( self, width = 500, height = 290, resolution = 0.01 ):
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
    self.canvas.pilimage = PILImage.fromarray( 255*np.ones( (height, width, 3), dtype = np.uint8 ) )
    self.canvas.bgimage = ImageTk.PhotoImage( self.canvas.pilimage )

    self.canvas.bind( '<ButtonPress-1>', self.click1 )
    self.canvas.bind( '<ButtonRelease-1>', self.click1_off )
    self.canvas.bind( '<B1-Motion>', self.click1_motion )
    self.canvas.bind( '<Key>', self.key_pressed )
    self.canvas.configure( cursor = 'left_ptr' )
    self.canvas.focus_set()

    signal.signal( signal.SIGINT, self.sigint_handler )

    rospy.Subscriber( 'real_pose', Pose, self.update_robot_pose )
    self.pub_map_metadata = rospy.Publisher( 'map_metadata', MapMetaData, queue_size = 1, latch = True )
    self.pub_map = rospy.Publisher( 'map', OccupancyGrid, queue_size = 1, latch = True )
    self.pub_pixel_pose = rospy.Publisher( 'pixel_pose', Pose, queue_size = 10 )

    map_quat = quaternion_from_euler( 0.0, 0.0, 0.0 )
    map_pose = Pose( Point( 0.0, height * self.resolution, 0.0 ), Quaternion( *map_quat ) )
    map_metadata = MapMetaData( rospy.Time.now(), resolution, width, height, map_pose )
    self.pub_map_metadata.publish( map_metadata )

    og_header = Header( 0, rospy.Time.now(), 'map_frame' )
    og_data = np.zeros( height * width, dtype = np.uint8 )
    occupancy_grid = OccupancyGrid( og_header, map_metadata, og_data.tolist() )
    self.pub_map.publish( occupancy_grid )

    self.statem = dict()
    self.statem['idle_mode'] = IdleMode( self.canvas )
    self.statem['set_robot_loc_mode'] = SetRobotLocationMode( self.canvas, self.pub_pixel_pose )
    self.statem['add_wall_mode'] = AddWallMode( self.canvas )
    self.statem['delete_wall_mode'] = DeleteWallMode( self.canvas )
    self.cstate = 'idle_mode'

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
    self.width, self.height = pilimage.size
    bgimage = ImageTk.PhotoImage( pilimage )

    self.root.geometry( '%dx%d' % (self.width, self.height) )
    # keep a reference to the image to avoid the image being garbage collected
    self.canvas.pilimage = pilimage
    self.canvas.bgimage = bgimage
    self.canvas.delete( 'backgroundimg' )
    self.canvas.config( width = self.width, height = self.height )
    self.canvas.create_image( 0, 0, anchor = NW, image = bgimage, tags = 'backgroundimg' )

    self.update_map()

  def click1( self, event ):
    self.statem[self.cstate].click1( event )

  def click1_off( self, event ):
    self.statem[self.cstate].click1_off( event )
    if self.cstate != 'idle_mode':
      self.cstate = 'idle_mode'
      self.canvas.config( cursor = 'left_ptr' )
    self.update_map()

  def click1_motion( self, event ):
    self.statem[self.cstate].click1_motion( event )

  def key_pressed( self, event ):
    if event.keysym == 'w' and self.cstate != 'add_wall_mode':
      self.cstate = 'add_wall_mode'
      self.canvas.config( cursor = 'pencil' )
    elif event.keysym == 'w' and self.cstate == 'add_wall_mode':
      self.cstate = 'idle_mode'
      self.canvas.config( cursor = 'left_ptr' )
    elif event.keysym == 'd' and self.cstate != 'delete_wall_mode':
      self.cstate = 'delete_wall_mode'
      self.canvas.config( cursor = 'X_cursor' )
    elif event.keysym == 'd' and self.cstate == 'delete_wall_mode':
      self.cstate = 'idle_mode'
      self.canvas.config( cursor = 'left_ptr' )
    #elif event.keysym == 'l' and self.cstate != 'set_robot_loc_mode':
    #  self.cstate = 'set_robot_loc_mode'
    #  self.canvas.config( cursor = 'hand2' )
    #elif event.keysym == 'l' and self.cstate == 'set_robot_loc_mode':
    #  self.cstate = 'idle_mode'
    #  self.canvas.config( cursor = 'left_ptr' )

  def update_robot_pose( self, pose ):
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

  def update_map( self ):
    background_image = self.canvas.pilimage.copy()
    draw = PILImageDraw.Draw( background_image )
    item_list = self.canvas.find_all()
    for item in item_list:
      objtype = self.canvas.type( item )
      if objtype == 'line':
        coords = self.canvas.coords( item )
        params = ['fill', 'tags', 'arrow']
        opt = dict()
        for p in params:
          opt[p] = self.canvas.itemcget( item, p )
        draw.line( coords, fill = opt['fill'] )

    width, height = background_image.size

    map_quat = quaternion_from_euler( 0.0, 0.0, 0.0 )
    map_pose = Pose( Point( 0.0, height * self.resolution, 0.0 ), Quaternion( *map_quat ) )
    map_metadata = MapMetaData( rospy.Time.now(), self.resolution, width, height, map_pose )
    self.pub_map_metadata.publish( map_metadata )

    og_header = Header( 0, rospy.Time.now(), 'map_frame' )
    og_data = np.array( background_image )
    if len( og_data.shape ) > 2:
      og_data = cv2.cvtColor( og_data, cv2.COLOR_BGR2GRAY )
    og_data = ( 100 - ( og_data / 255.0 ) * 100 ).astype( np.uint8 )
    og_data = og_data.reshape( height * width )
    occupancy_grid = OccupancyGrid( og_header, map_metadata, og_data.tolist() )
    self.pub_map.publish( occupancy_grid )

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


