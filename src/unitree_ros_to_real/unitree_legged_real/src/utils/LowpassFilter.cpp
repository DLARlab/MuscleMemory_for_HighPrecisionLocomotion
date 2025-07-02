# include "LowpassFilter.h"

int first_order_filter(float * FOF_frev, float * FOF_new, float para, int mun){

  float *fil_pre_value = FOF_frev;
  float *fil_new_value = FOF_new;

  for (int i = 0; i < mun; i++){
	     fil_pre_value[i] = fil_pre_value[i]*(1-para)+fil_new_value[i]*para;
  }
    
  return 0;
}