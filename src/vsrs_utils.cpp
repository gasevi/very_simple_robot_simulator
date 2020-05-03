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
CoordinateConverter::reset( float metric_zero_x, float metric_zero_y, float resolution )
{
  m_metric_zero_x = metric_zero_x;
  m_metric_zero_y = metric_zero_y;
  m_resolution = resolution;
}

pair<int, int>
CoordinateConverter::metric2pixel( float x, float y )
{
  int xpix =   static_cast<int>( ( x - m_metric_zero_x ) / m_resolution );
  int ypix = - static_cast<int>( ( y - m_metric_zero_y ) / m_resolution );
  return pair<int, int>( xpix, ypix );
}


