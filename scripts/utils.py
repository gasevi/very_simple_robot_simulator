import numpy as np


def sawtooth( x ):
  return (x-np.pi) % (2*np.pi) - np.pi

class CoordinateConverter( object ):

  def __init__( self, metric_zero_x, metric_zero_y, resolution ):
    self.metric_zero_x = metric_zero_x
    self.metric_zero_y = metric_zero_y
    self.resolution = resolution

  def metric2pixel( self, x, y ):
    xpix =   int( ( x - self.metric_zero_x ) / self.resolution )
    ypix = - int( ( y - self.metric_zero_y ) / self.resolution )
    return ( xpix, ypix )

  def pixel2metric( self, xpix, ypix ):
    x = xpix * self.resolution + self.metric_zero_x
    y = - ypix * self.resolution + self.metric_zero_y
    return ( x, y )


