#ifndef __VSRS_UTILS_H__
#define __VSRS_UTILS_H__

#include <vector>


std::vector<double> linspace( double start_in, double end, int num );

float sawtooth( float x );

class CoordinateConverter
{
public:

  CoordinateConverter( float origin_x = 0, float origin_y = 0, float resolution = 0, int map_height = 0 )
  : m_origin_x( origin_x ),
    m_origin_y( origin_y ),
    m_resolution( resolution ),
    m_map_height( map_height )
  {
  }

  void reset( float origin_x, float origin_y, float resolution, int map_height );

  std::pair<int, int> metric2pixel( float x, float y );

  std::pair<float, float> pixel2metric( int xpix, int ypix );

private:

  float m_origin_x;

  float m_origin_y;

  float m_resolution;

  int m_map_height;

};


#endif // __VSRS_UTILS_H__

