#ifndef __KOBUKI_SIMULATOR_H__
#define __KOBUKI_SIMULATOR_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

class KobukiSimulator
      : public rclcpp::Node
{
public:

  KobukiSimulator( std::string node_name,
                   float initial_x = 1.0,
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

  rclcpp::Node::SharedPtr m_node_handle;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_cmd_vel_sub;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_active_sub;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_initial_pose_sub;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_with_cov_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_real_pose_pub;

  void
  set_initial_pose( const geometry_msgs::Pose::ConstPtr& initial_pose );

  void
  set_initial_pose_with_cov( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initialpose );

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


