#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3, Point, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import threading
import numpy as np

class KobukiSimulator( Node ):

  def __init__( self, initial_x = 1.0, initial_y = 1.0, initial_yaw = 0.0 ):
    super().__init__( 'kobuki_simulator' )
    self.get_logger().info( 'Initializing Kobuki Simulator' )
    self.declare_parameter( 'initial_x', rclpy.Parameter.Type.DOUBLE )
    self.declare_parameter( 'initial_y', rclpy.Parameter.Type.DOUBLE )
    self.declare_parameter( 'initial_yaw', rclpy.Parameter.Type.DOUBLE )

    self.initial_x = initial_x
    try:
      self.initial_x = self.get_parameter( 'initial_x' ).get_parameter_value().double_value
    except rclpy.exceptions.ParameterUninitializedException:
      pass

    self.initial_y = initial_y
    try:
      self.initial_y = self.get_parameter( 'initial_y' ).get_parameter_value().double_value
    except rclpy.exceptions.ParameterUninitializedException:
      pass

    self.initial_yaw = initial_yaw
    try:
      self.initial_yaw = self.get_parameter( 'initial_yaw' ).get_parameter_value().double_value
    except rclpy.exceptions.ParameterUninitializedException:
      pass

    self.real_pose_publish_rate = 5.0 # [Hz]
    self.simulate_ground_friction = True
    self.reset = False

    # Odometry pose covariace
    self.cov_x = 1e-05
    self.cov_y = 1e-05
    self.cov_z = 1e-03

    pose_position = Point( x = self.initial_x, y = self.initial_y, z = 0.0 )
    q = quaternion_from_euler( 0.0, 0.0, self.initial_yaw )
    pose_quaternion = Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] )
    self.current_pose = Pose( position = pose_position, orientation = pose_quaternion )

    self.current_speed = Twist( linear = Vector3( x = 0.0, y = 0.0, z = 0.0 ),
                                angular = Vector3( x = 0.0, y = 0.0, z = 0.0 ) )

    self.odom_broadcaster = tf2_ros.TransformBroadcaster( self )
    self.create_subscription( Twist, '/cmd_vel', self.move, 1 )
    self.create_subscription( String, '/active', self.velocity_state, 1 )
    self.create_subscription( Pose, '/initial_pose', self.set_initial_pose, 1 )
    self.create_subscription( PoseWithCovarianceStamped, '/initialpose', self.set_initial_pose_with_cov, 1 )
    self.pub_odom = self.create_publisher( Odometry, '/odom', 1 )
    self.pub_real_pose = self.create_publisher( Pose, '/real_pose', 1 )

  def set_initial_pose( self, initial_pose ):
    if initial_pose.position.x == float('inf') and \
       initial_pose.position.y == float('inf'):
      q = quaternion_from_euler( 0.0, 0.0, self.initial_yaw )
      initial_pose = Pose( position = Point( x = self.initial_x, y = self.initial_y, z = 0.0 ),
                           orientation = Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] ) )
      self.reset = True
    self.current_pose = initial_pose
    self.pub_real_pose.publish( self.current_pose )

  def set_initial_pose_with_cov( self, initialpose ):
    self.current_pose = initialpose.pose.pose
    self.pub_real_pose.publish( self.current_pose )

  def update_real_pose( self, vx, vy, vyaw, dt ):
    roll, pitch, yaw = euler_from_quaternion( ( self.current_pose.orientation.x,
                                                self.current_pose.orientation.y,
                                                self.current_pose.orientation.z,
                                                self.current_pose.orientation.w ) )

    delta_x = (vx * np.cos( yaw ) - vy * np.sin( yaw )) * dt
    delta_y = (vx * np.sin( yaw ) + vy * np.cos( yaw )) * dt
    if self.simulate_ground_friction:
      delta_yaw = 0.9 * vyaw * dt
    else:
      delta_yaw = vyaw * dt

    x = self.current_pose.position.x + delta_x
    y = self.current_pose.position.y + delta_y
    yaw = yaw + delta_yaw

    q = quaternion_from_euler( roll, pitch, yaw )
    self.current_pose = Pose( position = Point( x = x, y = y, z = 0.0 ),
                              orientation = Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] ) )
    self.pub_real_pose.publish( self.current_pose )

  def publish_odom( self, x, y, yaw, vx, vy, vyaw, current_time ):
    t = TransformStamped()
    t.header.stamp = current_time.to_msg()
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_link'

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    q = quaternion_from_euler( 0.0, 0.0, yaw )
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    self.odom_broadcaster.sendTransform( t )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time.to_msg()
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'

    # set the position
    odom.pose.pose = Pose( position = Point( x = x, y = y, z = 0.0 ),
                           orientation = Quaternion( x = q[0], y = q[1], z = q[2], w = q[3] ) )

    odom.pose.covariance = [ self.cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                             0.0, self.cov_y, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 10000000.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 10000000.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 10000000.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, 0.0, self.cov_z ]

    # set the velocity
    odom.twist.twist = Twist( linear = Vector3( x = vx, y = vy, z = 0.0 ),
                              angular = Vector3( x = 0.0, y = 0.0, z = vyaw ) )

    odom.twist.covariance = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]

    self.pub_odom.publish( odom )

  def main_loop( self ):
    x = 0.0
    y = 0.0
    yaw = 0.0
    # Depends on incoming speed
    current_time = self.get_clock().now()
    last_time = self.get_clock().now()
    count = 1
    rate = self.create_rate( self.real_pose_publish_rate )
    try:
      while rclpy.ok():
        if self.reset:
          x, y, yaw = 0.0, 0.0, 0.0
          self.reset = False
        vx, vy, vyaw = self.get_current_speed()
        current_time = self.get_clock().now()
        dt = (current_time - last_time).nanoseconds * 1e-9

        self.update_real_pose( vx, vy, vyaw, dt )

        delta_x = (vx * np.cos( yaw ) - vy * np.sin( yaw )) * dt
        delta_y = (vx * np.sin( yaw ) + vy * np.cos( yaw )) * dt
        if self.simulate_ground_friction:
          delta_yaw = 0.9 * vyaw * dt
        else:
          delta_yaw = vyaw * dt

        x += delta_x
        y += delta_y
        yaw += delta_yaw

        # publish the message every 1 [s]
        #if count >= int( self.real_pose_publish_rate ):
        #  self.publish_odom( x, y, yaw, vx, vy, vyaw, current_time )
        #  count = 0
        #count += 1
        self.publish_odom( x, y, yaw, vx, vy, vyaw, current_time )

        last_time = current_time
        rate.sleep()
    except KeyboardInterrupt:
      self.get_logger().info( 'Shutting down kobuki_simulator' )
      pass

  def move( self, twist ):
    if np.isnan( twist.linear.x ) or np.isnan( twist.angular.z ):
      self.get_logger().warn( 'Invalid speed command received: lin.x: %s, ang.z: %s' % (str(twist.linear.x), str(twist.angular.z)) )
      return
    # movement is restricted to x and yaw
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    self.current_speed = twist

  def velocity_state( self, state ):
    if state.data == 'idle':
      self.current_speed = Twist( linear = Vector3( x = 0.0, y = 0.0, z = 0.0 ),
                                  angular = Vector3( x = 0.0, y = 0.0, z = 0.0 ) )

  def get_current_speed( self ):
    return self.current_speed.linear.x, self.current_speed.linear.y, self.current_speed.angular.z

  def shutdown( self ):
    # stop turtlebot
    self.get_logger().info( 'Stopping Kobuki Simulator' )
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.current_speed = Twist( linear = Vector3( x = 0.0, y = 0.0, z = 0.0 ),
                                angular = Vector3( x = 0.0, y = 0.0, z = 0.0 ) )
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep( 1 )


if __name__ == '__main__':
  rclpy.init()
  kobuki_sim = KobukiSimulator()

  thread = threading.Thread( target = rclpy.spin, args = (kobuki_sim, ), daemon = True )
  thread.start()

  kobuki_sim.main_loop()

  rclpy.shutdown()
  thread.join()


