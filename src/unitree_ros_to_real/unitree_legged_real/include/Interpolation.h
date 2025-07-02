#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>

float linear_interpolate( std::vector<float> &xData, std::vector<float> &yData, float x, bool extrapolate );

#endif
