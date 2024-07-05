#include "lidar_simulator.h"
#include "vsrs_utils.h"

#include <random>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/laser_scan.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using std::placeholders::_1;


LidarSimulator::LidarSimulator( string node_name, bool publish_2d_map )
: Node( node_name ),
  m_effective_hfov( kDefaultHFov ),
  m_map_resolution( 0.01 ),     // [m/pix]
  m_z_max( kDefaultViewDepth ), // [m]
  m_std_error( 0.005 ),         // [m]
  m_converter( 0, 0, m_map_resolution, 296 ),
  m_global_map( 296, 506, CV_8UC1, Scalar( 255 ) ),
  m_lidar_fov( M_PI ),
  m_lidar_n_h_scans( 181 ),
  m_image_transport( m_node_handle ),
  m_publish_2d_map( publish_2d_map )
{
  this->declare_parameter( "effective_hfov", rclcpp::PARAMETER_DOUBLE );
  this->declare_parameter( "view_depth", rclcpp::PARAMETER_DOUBLE );

  rclcpp::Parameter double_param;
  this->get_parameter_or( "effective_hfov", double_param, rclcpp::Parameter( "effective_hfov", kDefaultHFov ) );
  m_effective_hfov = double_param.as_double();
  this->get_parameter_or( "view_depth", double_param, rclcpp::Parameter( "view_depth", kDefaultViewDepth ) );
  m_z_max = double_param.as_double();

  m_hfov = m_effective_hfov*M_PI/180.0; // [rad]
  m_num_horizontal_scan = m_effective_hfov;

  m_view_depth_pix = static_cast<int>( m_z_max / m_map_resolution );
  m_horizontal_beam_angles = linspace( m_hfov/2.0, -m_hfov/2.0, m_num_horizontal_scan );

  m_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>( "world_map",
                                                                       1,
                                                                       std::bind(&LidarSimulator::set_map, this, _1) );
  m_real_pose_sub = this->create_subscription<geometry_msgs::msg::Pose>( "/real_pose",
                                                                         1,
                                                                         std::bind(&LidarSimulator::new_pose, this, _1) );
  m_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>( "/scan", 10 );

/*
  if( m_publish_2d_map )
  {
    m_lidar_2dmap_pub = m_image_transport.advertise( "camera/depth/lidar_in_map", 1 );
  }
*/
}

void
LidarSimulator::build_pixel_beam( int robot_pose_x,
                                  int robot_pose_y,
                                  float beam_angle,
                                  vector< pair<int, int> >& pixel_beam )
{
  int map_height = m_global_map.rows;
  int map_width = m_global_map.cols;
  int x0 = robot_pose_x;
  int y0 = robot_pose_y;
  float angle = sawtooth( beam_angle );
  int dx = 0;
  int dy = 0;

  if( abs( tan( angle ) ) > pow( 10, 4 ) )
  {
    if( angle > 0 )
    {
      dy = 0 - y0;
      dx = 0;
    }
    else
    {
      dy = -map_height - y0;
      dx = 0;
    }
  }
  else if( abs( tan( angle ) ) < pow( 10, -4 ) )
  {
    if( abs( angle ) < pow( 10, -4 ) )
    {
      dy = 0;
      dx = map_width - x0;
    }
    else
    {
      dy = 0;
      dx = 0 - x0;
    }
  }
  else
  {
    float a = tan( angle );
    float b = y0 - a*x0;
    float xp1 = -(b/a);
    float xp2 = -(map_height + b)/a;
    float yp1 = a*map_width + b;
    float yp2 = b;
    if( 0 <= angle && angle <= M_PI )
    {
      if( xp1 <= 0 )
      {
        dy = round( yp2 - y0 );
        dx = 0 - x0;
      }
      else if( 0 < xp1 && xp1 <= map_width )
      {
        dy = 0 - y0;
        dx = round( xp1 - x0 );
      }
      else
      {
        dy = round( yp1 - y0 );
        dx = map_width - x0;
      }
    }
    else if( -M_PI <= angle && angle < 0 )
    {
      if( xp2 <= 0 )
      {
        dy = round( yp2 - y0 );
        dx = 0 - x0;
      }
      else if( 0 < xp2 && xp2 <= map_width )
      {
        dy = -map_height - y0;
        dx = round( xp2 - x0 );
      }
      else
      {
        dy = round( yp1 - y0 );
        dx = map_width - x0;
      }
    }
  }

  int steps = 0;
  if( abs( dx ) > abs( dy ) )
  {
    steps = abs( dx );
  }
  else
  {
    steps = abs( dy );
  }

  if( steps > 0)
  {
    float x_inc = static_cast<float>( dx ) / steps;
    float y_inc = static_cast<float>( dy ) / steps;
    float beam_len = hypotf( dx, dy );
    float f = 1.0;
    if( beam_len > m_view_depth_pix )
    {
      f = m_view_depth_pix / beam_len;
    }

    float x = x0;
    float y = y0;
    int trimmed_steps = round( f * steps );
    for( int i = 0 ; i < trimmed_steps ; ++i )
    {
      if( 0 == m_global_map.at<uchar>( static_cast<int>( -y ), static_cast<int>( x ) ) )
      {
        break;
      }
      pixel_beam.push_back( pair<int, int>( static_cast<int>( -y ), static_cast<int>( x ) ) );
      x += x_inc;
      y += y_inc;
      if( x < 0 || map_width <= x || (-y) < 0 || map_height <= (-y) )
      {
        break;
      }
    }
  }
}

void
LidarSimulator::build_pixel_lidar( int robot_pose_x,
                                   int robot_pose_y,
                                   float robot_pose_yaw,
                                   vector<float>& distance_sensor )
{
  float yaw = sawtooth( robot_pose_yaw );
  float left_beam = yaw + m_hfov/2.0;
  float right_beam = yaw - m_hfov/2.0;
  vector< pair<int, int> > pixel_lidar;
  std::vector<double> lidar_angles = linspace( right_beam, left_beam, m_num_horizontal_scan );
  std::vector<double>::iterator ind;
  std::vector<double>::iterator end = lidar_angles.end();
  for( ind = lidar_angles.begin() ; ind != end ; ++ind )
  {
    float angle = *ind;
    vector< pair<int, int> > pixel_beam;
    build_pixel_beam( robot_pose_x, -robot_pose_y, angle, pixel_beam );
    float d = 0;
    if( pixel_beam.size() > 0 )
    {
      int dy = pixel_beam.back().first - pixel_beam.front().first;
      int dx = pixel_beam.back().second - pixel_beam.front().second;
      d = hypotf( dy, dx );
    }
    distance_sensor.push_back( m_map_resolution * d );
    pixel_lidar.insert( pixel_lidar.end(), pixel_beam.begin(), pixel_beam.end() );
  }

  if( m_publish_2d_map )
  {
    Mat map_to_send;
    cvtColor( m_global_map, map_to_send, CV_GRAY2RGB );
    vector< pair<int, int> >::iterator pixind;
    vector< pair<int, int> >::iterator pixend = pixel_lidar.end();
    for( pixind = pixel_lidar.begin() ; pixind != pixend ; ++pixind )
    {
       int y = pixind->first;
       int x = pixind->second;
       Vec3b& color = map_to_send.at<Vec3b>( y, x );
       color[0] = 255;
       color[1] = 0;
       color[2] = 0;
    }
    sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage( std_msgs::msg::Header(), "rgb8", map_to_send ).toImageMsg();
    m_lidar_2dmap_pub.publish( *image_msg.get() );
  }
}

void
LidarSimulator::new_pose( const geometry_msgs::msg::Pose::ConstPtr& pose_msg )
{
  pair<int, int> pix_pose = m_converter.metric2pixel( pose_msg->position.x, pose_msg->position.y );
  if( pix_pose.second < 0 || m_global_map.rows <= pix_pose.second ||
      pix_pose.first < 0 || m_global_map.cols <= pix_pose.first )
  {
    vector<float> scans( m_lidar_n_h_scans );
    fill( scans.begin(), scans.end(), m_z_max );
    send_laser_scan( scans );
    return;
  }

  tf2::Quaternion q( pose_msg->orientation.x,
                     pose_msg->orientation.y,
                     pose_msg->orientation.z,
                     pose_msg->orientation.w );
  tf2::Matrix3x3 m( q );
  double roll, pitch, yaw;
  m.getRPY( roll, pitch, yaw );

  int robot_pose_x = pix_pose.first;
  int robot_pose_y = pix_pose.second;
  float robot_pose_yaw = yaw;

  vector<float> distance_sensor;

  build_pixel_lidar( robot_pose_x, robot_pose_y, robot_pose_yaw, distance_sensor );

  random_device rd{};
  mt19937 gen{rd()};
  normal_distribution<float> dist{0, m_std_error};

  vector<float>::iterator ind;
  vector<float>::iterator end = distance_sensor.end();
  for( ind = distance_sensor.begin() ; ind != end ; ++ind )
  {
    *ind += dist(gen);        // add sensor noise
    *ind = *ind > m_z_max-0.2 ? m_z_max : *ind;
  }

  vector<float> scans( m_lidar_n_h_scans, m_z_max );
  int out_of_fov_beams = int( m_lidar_n_h_scans/2 ) - int( m_num_horizontal_scan/2 );
  if( out_of_fov_beams > 0 )
  {
    copy( distance_sensor.begin(), distance_sensor.end(), scans.begin() + out_of_fov_beams );
  }
  else
  {
    copy( distance_sensor.begin(), distance_sensor.end(), scans.begin() );
  }

  send_laser_scan( scans );
}

void
LidarSimulator::send_laser_scan( const vector<float>& scans )
{
  sensor_msgs::msg::LaserScan laserScan;
  laserScan.header.stamp = rclcpp::Clock().now();
  laserScan.header.frame_id = "base_link";
  laserScan.angle_min = -m_lidar_fov/2.0;
  laserScan.angle_max = m_lidar_fov/2.0;
  laserScan.angle_increment = 1.0*M_PI/180.0;
  laserScan.time_increment = 0.00001;
  laserScan.scan_time = 0.001*181;
  laserScan.range_min = 0.0;
  laserScan.range_max = m_z_max;

  laserScan.ranges.resize( scans.size() );
  laserScan.intensities.resize( scans.size() );
  for( unsigned int i = 0 ; i < scans.size() ; ++i )
  {
    laserScan.ranges[i] = scans[i];
    laserScan.intensities[i] = 0;
  }

  m_scan_pub->publish( laserScan );
}

void
LidarSimulator::set_map( const nav_msgs::msg::OccupancyGrid::ConstPtr& msg )
{
  std_msgs::msg::Header header = msg->header;
  nav_msgs::msg::MapMetaData info = msg->info;
  RCLCPP_INFO( get_logger(), "New map received (%d, %d, %f)", info.width, info.height, info.resolution );
  m_map_resolution = info.resolution;
  m_global_map.release();
  m_global_map.create( info.height, info.width, CV_8UC1 );
  for( unsigned int y = 0 ; y < info.height ; ++y )
  {
    for( unsigned int x = 0 ; x < info.width ; ++x )
    {
      // from occupancy grid to grayscale
      m_global_map.at<uchar>( info.height - 1 - y, x ) = ( 100 - static_cast<int>( msg->data[x + info.width * y] ) )*( 255/100.0 );
    }
  }
  m_converter.reset( info.origin.position.x, info.origin.position.y, info.resolution, info.height );
  m_view_depth_pix = static_cast<int>( m_z_max / m_map_resolution );
}


