import numpy as np

from utils import sawtooth


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
  elif abs( np.tan( angle ) ) < 10**(-4):
    if abs( angle ) < 10**(-4):
      dy = 0
      dx = w-x0
    else:
      dy = 0
      dx = 0 - x0
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
  pixel_beam = list()
  steps = abs(dx) if abs(dx) > abs(dy) else abs(dy)
  if steps > 0:
    x_inc = dx / float( steps )
    y_inc = dy / float( steps )
    beam_len = np.hypot( dx, dy )
    f = max_len/beam_len if beam_len > max_len else 1
    trimmed_steps = int( round( f*steps ) )
    x = x0
    y = y0
    for i in range( trimmed_steps ):
      if np.all( global_map[-int(y)][int(x)] == 0 ):
        break
      pixel_beam.append( [-int(y), int(x)] )
      x += x_inc
      y += y_inc
      if x >= w or y >= h:
        break
  return pixel_beam

def build_pixel_rangefinder( global_map, pose, fov, n_scans = 100, view_depth = 60 ):
  x, y, yaw = pose
  left_beam = yaw + fov/2.0
  right_beam = yaw - fov/2.0
  pixel_lidar = list()
  distance_sensor = list()
  for angle in np.linspace( right_beam, left_beam, n_scans ).tolist():
    robot_pose = (x, y, angle)
    pixel_beam = build_pixel_beam( global_map, robot_pose, view_depth )
    pixel_lidar.append( pixel_beam )
    d = 0
    if len( pixel_beam ) > 0:
      d = np.sqrt( (pixel_beam[-1][0] - pixel_beam[0][0])**2 + (pixel_beam[-1][1] - pixel_beam[0][1])**2 )
    distance_sensor.append( d )
  return pixel_lidar, distance_sensor


