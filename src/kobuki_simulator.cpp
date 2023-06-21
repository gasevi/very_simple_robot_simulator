#include "kobuki_simulator.h"

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;


KobukiSimulator::KobukiSimulator( float initial_x,
                                  float initial_y,
                                  float initial_yaw )
: m_initial_x( initial_x ),
  m_initial_y( initial_y ),
  m_initial_yaw( initial_yaw ),
  m_simulate_ground_friction( true ),
  m_reset( true )
{
  ROS_INFO( "Initializing Kobuki Simulator" );

  m_node_handle.param<float>( "/kobuki_simulator/initial_x", m_initial_x, initial_x );
  m_node_handle.param<float>( "/kobuki_simulator/initial_y", m_initial_y, initial_y );
  m_node_handle.param<float>( "/kobuki_simulator/initial_yaw", m_initial_yaw, initial_yaw );

  m_current_pose.position.x = m_initial_x;
  m_current_pose.position.y = m_initial_y;
  m_current_pose.position.z = 0;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw( m_initial_yaw );
  m_current_pose.orientation = quat;

  m_cmd_vel_sub = m_node_handle.subscribe( "cmd_vel", 1, &KobukiSimulator::move, this );
  m_active_sub = m_node_handle.subscribe( "cmd_vel_active", 1, &KobukiSimulator::velocity_state, this );
  //m_cmd_vel_sub = m_node_handle.subscribe( "yocs_cmd_vel_mux/output/cmd_vel", 1, &KobukiSimulator::move, this );
  //m_active_sub = m_node_handle.subscribe( "yocs_cmd_vel_mux/active", 1, &KobukiSimulator::velocity_state, this );
  m_initial_pose_sub = m_node_handle.subscribe( "initial_pose", 1, &KobukiSimulator::set_initial_pose, this );
  m_initial_pose_with_cov_sub = m_node_handle.subscribe( "initialpose", 1, &KobukiSimulator::set_initial_pose_with_cov, this );
  m_odom_pub = m_node_handle.advertise<nav_msgs::Odometry>( "odom", 10 );
  m_real_pose_pub = m_node_handle.advertise<geometry_msgs::Pose>( "real_pose", 1 );
}

void
KobukiSimulator::set_initial_pose( const geometry_msgs::Pose::ConstPtr& initial_pose )
{
  if( initial_pose->position.x == INFINITY || initial_pose->position.y == INFINITY )
  {
    m_current_pose.position.x = m_initial_x;
    m_current_pose.position.y = m_initial_y;
    m_current_pose.position.z = 0;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw( m_initial_yaw );
    m_current_pose.orientation = quat;
    m_reset = true;
  }
  else
  {
    m_current_pose.position = initial_pose->position;
    m_current_pose.orientation = initial_pose->orientation;
  }
  ROS_INFO( "Initial pose received: (%.2f, %.2f)",
            m_current_pose.position.x,
            m_current_pose.position.y );
  m_real_pose_pub.publish( m_current_pose );
}

void
KobukiSimulator::set_initial_pose_with_cov( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initialpose )
{
  m_current_pose = initialpose->pose.pose;
  ROS_INFO( "Initial pose received: (%.2f, %.2f)",
            m_current_pose.position.x,
            m_current_pose.position.y );
  m_real_pose_pub.publish( m_current_pose );
}

void
KobukiSimulator::update_real_pose( float vx, float vy, float vyaw, float dt )
{
  tf::Quaternion q( m_current_pose.orientation.x,
                    m_current_pose.orientation.y,
                    m_current_pose.orientation.z,
                    m_current_pose.orientation.w );
  tf::Matrix3x3 m( q );
  double roll, pitch, yaw;
  m.getRPY( roll, pitch, yaw );

  float delta_x = (vx * cos( yaw + m_initial_yaw ) - vy * sin( yaw + m_initial_yaw )) * dt;
  float delta_y = (vx * sin( yaw + m_initial_yaw ) + vy * cos( yaw + m_initial_yaw )) * dt;
  float delta_yaw = 0;
  if( m_simulate_ground_friction )
  {
    delta_yaw = 0.9 * vyaw * dt;
  }
  else
  {
    delta_yaw = vyaw * dt;
  }

  float x = m_current_pose.position.x + delta_x;
  float y = m_current_pose.position.y + delta_y;
  yaw = yaw + delta_yaw;

  m_current_pose.position.x = x;
  m_current_pose.position.y = y;
  m_current_pose.position.z = 0;
  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw( yaw );
  m_current_pose.orientation = quat;

  m_real_pose_pub.publish( m_current_pose );
}

void
KobukiSimulator::publish_odom( float x, float y, float yaw, float vx, float vy, float vyaw, const ros::Time& current_time )
{
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( yaw );
  odom_trans.transform.rotation = odom_quat;
  m_odom_broadcaster.sendTransform( odom_trans );

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vyaw;

  //publish the message
  m_odom_pub.publish( odom );
}

void
KobukiSimulator::main_loop()
{
  float x = 0;
  float y = 0;
  float yaw = 0;
  float vx = 0;
  float vy = 0;
  float vyaw = 0;
  float delta_x = 0;
  float delta_y = 0;
  float delta_yaw = 0;
  float dt = 0;
  // Depends on incoming speed
  ros::Time current_time = ros::Time::now();
  ros::Time last_time = ros::Time::now();
  int count = 1;

  ros::Rate rate( kRealPosePublishRate );
  while( ros::ok() )
  {
    ros::spinOnce();
    if( m_reset )
    {
      x, y, yaw = 0, 0, 0;
      m_reset = false;
    }

    get_current_speed( vx, vy, vyaw );
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();

    update_real_pose( vx, vy, vyaw, dt );

    delta_x = (vx * cos( yaw + m_initial_yaw ) - vy * sin( yaw + m_initial_yaw )) * dt;
    delta_y = (vx * sin( yaw + m_initial_yaw ) + vy * cos( yaw + m_initial_yaw )) * dt;
    if( m_simulate_ground_friction )
    {
      delta_yaw = 0.9 * vyaw * dt;
    }
    else
    {
      delta_yaw = vyaw * dt;
    }

    x += delta_x;
    y += delta_y;
    yaw += delta_yaw;

    /*
    // publish the message every 1 [s]
    if( count >= kRealPosePublishRate )
    {
      publish_odom( x, y, yaw, vx, vy, vyaw, current_time );
      count = 0;
    }
    count += 1;
    */
    publish_odom( x, y, yaw, vx, vy, vyaw, current_time );

    last_time = current_time;
    rate.sleep();
  }
}

void
KobukiSimulator::move( const geometry_msgs::Twist& twist )
{
  if( twist.linear.x == NAN || twist.angular.z == NAN )
  {
    ROS_WARN( "Invalid speed command received: lin.x: %f, ang.z: %f", twist.linear.x, twist.angular.z );
    return;
  }
  // movement is restricted to x and yaw
  m_current_speed.linear.x = twist.linear.x;
  m_current_speed.angular.z = twist.angular.z;
}

void
KobukiSimulator::velocity_state( const std_msgs::String& state )
{
  ROS_INFO( "Current subscriptor: %s", state.data.c_str() );
  if( "idle" == state.data )
  {
    m_current_speed.linear.x = 0;
    m_current_speed.angular.z = 0;
  }
}

void
KobukiSimulator::get_current_speed( float& vx, float& vy, float& vyaw )
{
  vx = m_current_speed.linear.x;
  vy = m_current_speed.linear.y;
  vyaw = m_current_speed.angular.z;
}


