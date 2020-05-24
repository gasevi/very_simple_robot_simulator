#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Quaternion, Vector3, Point
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

class KobukiSimulator( object ):

  def __init__( self, initial_x = 1.0, initial_y = 1.0, initial_yaw = 0.0 ):
    rospy.loginfo( 'Initializing Kobuki Simulator' )
    rospy.on_shutdown( self.shutdown )

    self.initial_x = initial_x
    self.initial_y = initial_y
    self.initial_yaw = initial_yaw
    self.real_pose_publish_rate = 5.0 # [Hz]
    self.simulate_ground_friction = True
    self.reset = False

    odom_quat = quaternion_from_euler( 0.0, 0.0, self.initial_yaw )
    self.current_pose = Pose( Point( self.initial_x, self.initial_y, 0.0 ), Quaternion( *odom_quat ) )
    self.current_speed = Twist( Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) )

    self.odom_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber( 'yocs_cmd_vel_mux/output/cmd_vel', Twist, self.move )
    rospy.Subscriber( 'yocs_cmd_vel_mux/active', String, self.velocity_state )
    rospy.Subscriber( 'initial_pose', Pose, self.set_initial_pose )
    self.pub_odom = rospy.Publisher( 'odom', Odometry, queue_size = 10 )
    self.pub_real_pose = rospy.Publisher( 'real_pose', Pose, queue_size = 1 )

  def set_initial_pose( self, initial_pose ):
    if initial_pose.position.x == float('inf') and \
       initial_pose.position.y == float('inf'):
      odom_quat = quaternion_from_euler( 0.0, 0.0, self.initial_yaw )
      initial_pose = Pose( Point( self.initial_x, self.initial_y, 0.0 ), Quaternion( *odom_quat ) )
      self.reset = True
    self.current_pose = initial_pose
    self.pub_real_pose.publish( self.current_pose )

  def update_real_pose( self, vx, vy, vyaw, dt ):
    roll, pitch, yaw = euler_from_quaternion( ( self.current_pose.orientation.x,
                                                self.current_pose.orientation.y,
                                                self.current_pose.orientation.z,
                                                self.current_pose.orientation.w ) )

    delta_x = (vx * np.cos( yaw + self.initial_yaw ) - vy * np.sin( yaw + self.initial_yaw )) * dt
    delta_y = (vx * np.sin( yaw + self.initial_yaw ) + vy * np.cos( yaw + self.initial_yaw )) * dt
    if self.simulate_ground_friction:
      delta_yaw = 0.9 * vyaw * dt
    else:
      delta_yaw = vyaw * dt

    x = self.current_pose.position.x + delta_x
    y = self.current_pose.position.y + delta_y
    yaw = yaw + delta_yaw

    quat = quaternion_from_euler( roll, pitch, yaw )
    self.current_pose = Pose( Point( x, y, 0.0 ), Quaternion( *quat ) )
    self.pub_real_pose.publish( self.current_pose )

  def publish_odom( self, x, y, yaw, vx, vy, vyaw, current_time ):
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = quaternion_from_euler( 0.0, 0.0, yaw )

    # first, we'll publish the transform over tf
    self.odom_broadcaster.sendTransform(
                                         (x, y, 0.0),
                                         odom_quat,
                                         current_time,
                                         'base_link',
                                         'odom'
                                       )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'

    # set the position
    odom.pose.pose = Pose( Point( x, y, 0.0 ), Quaternion( *odom_quat ) )

    # set the velocity
    odom.child_frame_id = 'base_link'
    odom.twist.twist = Twist( Vector3( vx, vy, 0.0 ), Vector3( 0.0, 0.0, vyaw ) )

    self.pub_odom.publish( odom )

  def main_loop( self ):
    x = 0.0
    y = 0.0
    yaw = 0.0
    # Depends on incoming speed
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    count = 1
    rate = rospy.Rate( self.real_pose_publish_rate )
    while not rospy.is_shutdown():
      if self.reset:
        x, y, yaw = 0.0, 0.0, 0.0
        self.reset = False
      vx, vy, vyaw = self.get_current_speed()
      current_time = rospy.Time.now()
      dt = (current_time - last_time).to_sec()

      self.update_real_pose( vx, vy, vyaw, dt )

      delta_x = (vx * np.cos( yaw + self.initial_yaw ) - vy * np.sin( yaw + self.initial_yaw )) * dt
      delta_y = (vx * np.sin( yaw + self.initial_yaw ) + vy * np.cos( yaw + self.initial_yaw )) * dt
      if self.simulate_ground_friction:
        delta_yaw = 0.9 * vyaw * dt
      else:
        delta_yaw = vyaw * dt

      x += delta_x
      y += delta_y
      yaw += delta_yaw

      # publish the message every 1 [s]
      if count >= int( self.real_pose_publish_rate ): 
        self.publish_odom( x, y, yaw, vx, vy, vyaw, current_time )
        count = 0
      count += 1

      last_time = current_time
      rate.sleep()

  def move( self, twist ):
    if np.isnan( twist.linear.x ) or np.isnan( twist.angular.z ):
      rospy.logwarn( 'Invalid speed command received: lin.x: %s, ang.z: %s' % (str(twist.linear.x), str(twist.angular.z)) )
      return
    # movement is restricted to x and yaw
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    self.current_speed = twist

  def velocity_state( self, state ):
    rospy.loginfo( 'Current subscriptor: %s' % state.data )
    if state.data == 'idle':
      self.current_speed = Twist( Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) )

  def get_current_speed( self ):
    return self.current_speed.linear.x, self.current_speed.linear.y, self.current_speed.angular.z

  def shutdown( self ):
    # stop turtlebot
    rospy.loginfo( 'Stopping Kobuki Simulator' )
    # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
    self.current_speed = Twist( Vector3( 0.0, 0.0, 0.0 ), Vector3( 0.0, 0.0, 0.0 ) )
    # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep( 1 )


if __name__ == '__main__':
  rospy.init_node( 'kobuki_simulator' )
  kobuki_sim = KobukiSimulator()
  kobuki_sim.main_loop()



