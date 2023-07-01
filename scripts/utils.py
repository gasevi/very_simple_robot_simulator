import numpy as np


def sawtooth( x ):
  return (x-np.pi) % (2*np.pi) - np.pi

class CoordinateConverter( object ):

  def __init__( self, origin_x, origin_y, resolution, map_height ):
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.resolution = resolution
    self.map_height = map_height

  def update( self, origin_x, origin_y, resolution, map_height ):
    self.origin_x = origin_x
    self.origin_y = origin_y
    self.resolution = resolution
    self.map_height = map_height

  def metric2pixel( self, x, y ):
    xpix =   int( ( x - self.origin_x ) / self.resolution )
    ypix = - int( ( y - self.origin_y ) / self.resolution ) + self.map_height
    return ( xpix, ypix )

  def pixel2metric( self, xpix, ypix ):
    x = xpix * self.resolution + self.origin_x
    y = ( self.map_height - ypix ) * self.resolution + self.origin_y
    return ( x, y )


