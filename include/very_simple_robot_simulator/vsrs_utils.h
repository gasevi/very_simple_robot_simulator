#ifndef __VSRS_UTILS_H__
#define __VSRS_UTILS_H__

#include <vector>


std::vector<double> linspace( double start_in, double end, int num );

float sawtooth( float x );

class CoordinateConverter
{
public:

  CoordinateConverter( float metric_zero_x = 0, float metric_zero_y = 0, float resolution = 0, int map_height = 0 )
  : m_metric_zero_x( metric_zero_x ),
    m_metric_zero_y( metric_zero_y ),
    m_resolution( resolution ),
    m_map_height( map_height )
  {
  }

  void reset( float metric_zero_x, float metric_zero_y, float resolution, int map_height );

  std::pair<int, int> metric2pixel( float x, float y );

  std::pair<float, float> pixel2metric( int xpix, int ypix );

private:

  float m_metric_zero_x;

  float m_metric_zero_y;

  float m_resolution;

  int m_map_height;

};


#endif // __VSRS_UTILS_H__

