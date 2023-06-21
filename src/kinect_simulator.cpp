#include "kinect_simulator.h"
#include "vsrs_utils.h"

#include <ros/console.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;


KinectSimulator::KinectSimulator( bool publish_2d_map )
: m_global_map( 296, 506, CV_8UC1, Scalar( 255 ) ),
  m_map_resolution( 0.01 ),
  m_converter( 0, 0, m_map_resolution, 296 ),
  m_image_transport( m_node_handle ),
  m_publish_2d_map( publish_2d_map )
{
  m_num_horizontal_scan = 50;
  m_num_vertical_scan = static_cast<int>( ( kDepthImgHeight * m_num_horizontal_scan ) / kDepthImgWidth );
  m_view_depth_pix = static_cast<int>( kViewDepth / m_map_resolution );
  m_horizontal_beam_angles = linspace( kHfov / 2.0, - kHfov / 2.0, m_num_horizontal_scan );
  m_vertical_beam_angles = linspace( kVfov / 2.0, - kVfov / 2.0, m_num_vertical_scan );

  m_real_pose_sub = m_node_handle.subscribe( "real_pose", 1, &KinectSimulator::new_pose, this );
  m_map_sub = m_node_handle.subscribe( "map", 1, &KinectSimulator::set_map, this );
  m_depth_image_pub = m_image_transport.advertise( "camera/depth/image_raw", 1 );
  if( m_publish_2d_map )
  {
    m_lidar_2dmap_pub = m_image_transport.advertise( "camera/depth/lidar_in_map", 1 );
  }
}

void
KinectSimulator::build_pixel_beam( int robot_pose_x,
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
KinectSimulator::build_pixel_lidar( int robot_pose_x,
                                    int robot_pose_y,
                                    float robot_pose_yaw,
                                    vector<float>& distance_sensor )
{
  float yaw = sawtooth( robot_pose_yaw );
  float left_beam = yaw + kHfov/2.0;
  float right_beam = yaw - kHfov/2.0;
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
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage( std_msgs::Header(), "rgb8", map_to_send ).toImageMsg();
    m_lidar_2dmap_pub.publish( image_msg );
  }
}

void
KinectSimulator::new_pose( const geometry_msgs::Pose::ConstPtr& pose_msg )
{
  Mat depth_image( m_num_vertical_scan, m_num_horizontal_scan, CV_32FC1, Scalar( kMaxDepth ) );

  pair<int, int> pix_pose = m_converter.metric2pixel( pose_msg->position.x, pose_msg->position.y );
  if( pix_pose.second < 0 || m_global_map.rows <= pix_pose.second ||
      pix_pose.first < 0 || m_global_map.cols <= pix_pose.first )
  {
    Size new_size( kDepthImgWidth, kDepthImgHeight );
    Mat depth_image_resized;
    resize( depth_image, depth_image_resized, new_size );

    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage( std_msgs::Header(), "32FC1", depth_image_resized ).toImageMsg();
    m_depth_image_pub.publish( image_msg );
    return;
  }

  tf::Quaternion q( pose_msg->orientation.x,
                    pose_msg->orientation.y,
                    pose_msg->orientation.z,
                    pose_msg->orientation.w );
  tf::Matrix3x3 m( q );
  double roll, pitch, yaw;
  m.getRPY( roll, pitch, yaw );

  int robot_pose_x = pix_pose.first;
  int robot_pose_y = pix_pose.second;
  float robot_pose_yaw = yaw;

  vector<float> distance_sensor;

  build_pixel_lidar( robot_pose_x, robot_pose_y, robot_pose_yaw, distance_sensor );

  for( int c = 0 ; c < m_num_horizontal_scan ; ++c )
  {
    float d = distance_sensor[m_num_horizontal_scan-c-1];
    d = d * cos( m_horizontal_beam_angles[c] ); // project beam into the robot plane

    float ceiling_angle = atan2( kWallHeight - kKinectHeight, d );
    int ceiling_limit_index = -1;
    if( ceiling_angle < kVfov / 2.0 )
    {
      for( int r = 0 ; r < m_num_vertical_scan ; ++r )
      {
        if( m_vertical_beam_angles[r] > ceiling_angle )
        {
          float& depth = depth_image.at<float>( r, c );
          depth = kMaxDepth;
          ceiling_limit_index = r;
        }
      }
    }

    float ground_angle = atan2( kKinectHeight, d );
    int ground_limit_index = m_num_vertical_scan;
    if( ground_angle < kVfov / 2.0 )
    {
      for( int r = m_num_vertical_scan - 1 ; r >= 0 ; --r )
      { 
        if( m_vertical_beam_angles[r] < -ground_angle )
        { 
          float ground_d = kKinectHeight / sin( abs( m_vertical_beam_angles[r] ) );
          float& depth = depth_image.at<float>( r, c );
          if( ground_d >= kMinValidDistance )
          {
            depth = ground_d;
          }
          else
          {
            depth = NAN;
          }
          ground_limit_index = r;
        }
      }
    }

    int start_row = ceiling_limit_index < 0 ? 0 : ceiling_limit_index + 1;
    for( int r = start_row ; r < ground_limit_index ; ++r )
    {
      float& depth_pixel = depth_image.at<float>( r, c );
      depth_pixel = d >= kMinValidDistance ? d : NAN;
    }
  }

  Size new_size( kDepthImgWidth, kDepthImgHeight );
  Mat depth_image_resized;
  resize( depth_image, depth_image_resized, new_size );

  sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage( std_msgs::Header(), "32FC1", depth_image_resized ).toImageMsg();
  m_depth_image_pub.publish( image_msg );
}

void
KinectSimulator::set_map( const nav_msgs::OccupancyGrid::ConstPtr& msg )
{
  std_msgs::Header header = msg->header;
  nav_msgs::MapMetaData info = msg->info;
  ROS_INFO( "New map received (%d, %d, %f)", info.width, info.height, info.resolution );
  m_map_resolution = info.resolution;
  for( unsigned int y = 0 ; y < info.height ; ++y )
  {
    for( unsigned int x = 0 ; x < info.width ; ++x )
    {
      // from occupancy grid to grayscale
      m_global_map.at<uchar>( info.height - 1 - y, x ) = ( 100 - static_cast<int>( msg->data[x + info.width * y] ) )*( 255/100.0 );
    }
  }
  m_converter.reset( info.origin.position.x, info.origin.position.y, info.resolution, info.height );
  m_view_depth_pix = static_cast<int>( kViewDepth / m_map_resolution );
}


