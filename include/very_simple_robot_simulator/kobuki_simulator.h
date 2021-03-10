#ifndef __KOBUKI_SIMULATOR_H__
#define __KOBUKI_SIMULATOR_H__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class KobukiSimulator
{
public:

  KobukiSimulator( float initial_x = 1.0,
                   float initial_y = 1.0,
                   float initial_yaw = 0.0 );

  void
  main_loop();

private:

  static
  constexpr float kRealPosePublishRate = 5.0; // [Hz]

  float m_initial_x;

  float m_initial_y;

  float m_initial_yaw;

  tf::TransformBroadcaster m_odom_broadcaster;

  geometry_msgs::Pose m_current_pose;

  geometry_msgs::Twist m_current_speed;

  bool m_simulate_ground_friction;

  bool m_reset;

  ros::NodeHandle m_node_handle;

  ros::Subscriber m_cmd_vel_sub;

  ros::Subscriber m_active_sub;

  ros::Subscriber m_initial_pose_sub;

  ros::Publisher m_odom_pub;

  ros::Publisher m_real_pose_pub;

  void
  set_initial_pose( const geometry_msgs::Pose::ConstPtr& initial_pose );

  void
  update_real_pose( float vx, float vy, float vyaw, float dt );

  void
  publish_odom( float x, float y, float yaw, float vx, float vy, float vyaw, const ros::Time& current_time );

  void
  move( const geometry_msgs::Twist& twist );

  void
  velocity_state( const std_msgs::String& state );

  void
  get_current_speed( float& vx, float& vy, float& vyaw );

};

#endif // __KOBUKI_SIMULATOR_H__


