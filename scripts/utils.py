
class CoordinateConverter( object ):

  def __init__( self, cartesian_zero_x, cartesian_zero_y, resolution ):
    self.cartesian_zero_x = cartesian_zero_x
    self.cartesian_zero_y = cartesian_zero_y
    self.resolution = resolution

  def cartesian2pixel( self, x, y ):
    xpix =   int( ( x - self.cartesian_zero_x ) / self.resolution )
    ypix = - int( ( y - self.cartesian_zero_y ) / self.resolution )
    return ( xpix, ypix )



