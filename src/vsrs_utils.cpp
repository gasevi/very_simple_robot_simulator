#include "vsrs_utils.h"

#include <cmath>

using namespace std;


std::vector<double> linspace( double start, double end, int num )
{
  std::vector<double> linspaced;

  if( num == 0 )
  {
    return linspaced;
  }

  if( num == 1 )
  {
    linspaced.push_back( start );
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for( int i = 0 ; i < num-1 ; ++i )
  {
    linspaced.push_back( start + delta * i );
  }

  linspaced.push_back( end );

  return linspaced;
}

float sawtooth( float x )
{
  return remainder( x, 2 * M_PI );
}


void
CoordinateConverter::reset( float origin_x, float origin_y, float resolution, int map_height )
{
  m_origin_x = origin_x;
  m_origin_y = origin_y;
  m_resolution = resolution;
  m_map_height = map_height;
}

pair<int, int>
CoordinateConverter::metric2pixel( float x, float y )
{
  int xpix =   static_cast<int>( ( x - m_origin_x ) / m_resolution );
  int ypix = - static_cast<int>( ( y - m_origin_y ) / m_resolution ) + m_map_height;
  return pair<int, int>( xpix, ypix );
}

pair<float, float>
CoordinateConverter::pixel2metric( int xpix, int ypix )
{
  float x = xpix * m_resolution + m_origin_x;
  float y = (m_map_height - ypix) * m_resolution + m_origin_y;
  return pair<float, float>( x, y );
}


