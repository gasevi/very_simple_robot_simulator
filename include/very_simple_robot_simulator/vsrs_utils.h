#ifndef __VSRS_UTILS_H__
#define __VSRS_UTILS_H__

#include <vector>


std::vector<double> linspace( double start_in, double end, int num );

float sawtooth( float x );

class CoordinateConverter
{
public:

  CoordinateConverter( float metric_zero_x = 0, float metric_zero_y = 0, float resolution = 0 )
  : m_metric_zero_x( metric_zero_x ),
    m_metric_zero_y( metric_zero_y ),
    m_resolution( resolution )
  {
  }

  void reset( float metric_zero_x, float metric_zero_y, float resolution );

  std::pair<int, int> metric2pixel( float x, float y );

private:

  float m_metric_zero_x;

  float m_metric_zero_y;

  float m_resolution;

};


#endif // __VSRS_UTILS_H__

