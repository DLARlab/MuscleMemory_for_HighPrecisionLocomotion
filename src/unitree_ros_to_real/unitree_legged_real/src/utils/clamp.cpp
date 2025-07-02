#include <algorithm>

#include "clamp.h"

float clamp(float a,  float lim1, float lim2)
{
  //call by value//campare two limits
  float a_min, a_max, b;
  a_min = std::min<float>( lim1, lim2 );
  a_max = std::max<float>( lim1, lim2 );

  b = std::max<float>(std::min<float>(a, a_max), a_min) ;

  return b;
}
