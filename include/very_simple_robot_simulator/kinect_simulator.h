#ifndef __KINECT_SIMULATOR_H__
#define __KINECT_SIMULATOR_H__

#include "vsrs_utils.h"

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>

class KinectSimulator
      : public rclcpp::Node
{
public:

  KinectSimulator( std::string node_name, bool publish_2d_map = false );

private:

  static
  constexpr float kKinectHeight = 0.3; // [m]

  static
  constexpr float kWallHeight = 0.5; // [m]

  static
  constexpr float kMaxDepth = 10.0; // [m]

  static
  constexpr float kHfov = 0.995; // [rad] (57 [degrees])

  static
  constexpr float kVfov = 0.750; // [rad] (43 [degrees])

  static
  constexpr float kViewDepth = 4.0; // [m]

  static
  constexpr float kDepthImgWidth = 640; // [pix]

  static
  constexpr float kDepthImgHeight = 480; // [pix]

  static
  constexpr float kMinValidDistance = 0.45; // [m]

  cv::Mat m_global_map;

  float m_map_resolution; // [m/pix]

  CoordinateConverter m_converter;

  int m_num_horizontal_scan;

  int m_num_vertical_scan;

  int m_view_depth_pix;

  std::vector<double> m_horizontal_beam_angles;

  std::vector<double> m_vertical_beam_angles;

  rclcpp::Node::SharedPtr m_node_handle;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_real_pose_sub;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_map_sub;

  image_transport::ImageTransport m_image_transport;

  image_transport::Publisher m_depth_image_pub;

  image_transport::Publisher m_lidar_2dmap_pub;

  bool m_publish_2d_map;

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

};

#endif // __KINECT_SIMULATOR_H__

