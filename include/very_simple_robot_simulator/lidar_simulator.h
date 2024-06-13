#ifndef __LIDAR_SIMULATOR_H__
#define __LIDAR_SIMULATOR_H__

#include "vsrs_utils.h"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

class LidarSimulator
      : public rclcpp::Node
{
public:

  LidarSimulator( std::string node_name, bool publish_2d_map = false );

private:

  static
  constexpr float kDefaultHFov = 57; // [deg]

  static
  constexpr float kDefaultViewDepth = 4.0; // [m]

  float m_effective_hfov;

  cv::Mat m_global_map;

  float m_map_resolution; // [m/pix]

  CoordinateConverter m_converter;

  int m_num_horizontal_scan;

  float m_hfov;

  int m_lidar_n_h_scans;

  float m_lidar_fov;

  float m_z_max;

  int m_view_depth_pix;

  int m_seq;

  float m_std_error;

  bool m_publish_2d_map;

  std::vector<double> m_horizontal_beam_angles;

  rclcpp::Node::SharedPtr m_node_handle;

  image_transport::ImageTransport m_image_transport;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_real_pose_sub;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_pub;

  image_transport::Publisher m_lidar_2dmap_pub;

  void
  build_pixel_beam( int robot_pose_x,
                    int robot_pose_y,
                    float beam_angle,
                    std::vector< std::pair<int, int> >& pixel_beam );

  void
  build_pixel_lidar( int robot_pose_x,
                     int robot_pose_y,
                     float robot_pose_yaw,
                     std::vector<float>& distance_sensor );

  void new_pose( const geometry_msgs::msg::Pose::ConstPtr& pose_msg );

  void set_map( const nav_msgs::msg::OccupancyGrid::ConstPtr& msg );

  void send_laser_scan( const std::vector<float>& msg );

};

#endif // __LIDAR_SIMULATOR_H__

