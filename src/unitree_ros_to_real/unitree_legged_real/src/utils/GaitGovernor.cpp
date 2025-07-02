#include <algorithm>
#include <math.h>

#include "GaitGovernor.h"

float velocity_regulation(float& fly_sel_velocity, float& transition_rate_temp, float (&desired_velocity_trunk)[3], float (&trunk_Vel_strongfil)[3], long motiontime)
{
    float difference_sel_vel, difference_vel_realtime, difference_selv_integral, hd_FR_LA, Kp_theta, Kd_theta, para_hd_FR;

    /////////////////////////////////////////////////////////////////
    ///////desired_velocity_trunk[0] =  0.0;/////////////////////////
    difference_sel_vel = fly_sel_velocity - desired_velocity_trunk[0];
    difference_vel_realtime = trunk_Vel_strongfil[0] - desired_velocity_trunk[0];
               
    if (transition_rate_temp < 1){
         difference_selv_integral = 0.0;
         Kd_theta =  2.0;        /////////////undecided         
    }
    else{
        Kd_theta =  2.0;
        if (abs(difference_vel_realtime) < 0.1){
            difference_selv_integral = difference_selv_integral + 0.2*difference_sel_vel;
        }
        else{
            difference_selv_integral = 0.0;
        }
    }

    if (abs(difference_vel_realtime) < 0.1){
        //  Kp_theta = 3.0;
        Kp_theta =  0.5*abs(3.0 - 30*abs(difference_vel_realtime));
    }
    else{
        Kp_theta =  0.0;
    }
        // Kd_theta =  2.0 - 5*abs(difference_sel_vel);
    if (fabs(difference_sel_vel) > 0.1){
        difference_sel_vel = copysign(0.1, difference_sel_vel);
    } 
    else{
        difference_sel_vel = difference_sel_vel;
    }
                
        hd_FR_LA =Kp_theta * difference_vel_realtime + Kd_theta * difference_sel_vel +  1.0*0.02 * difference_selv_integral;

    if (fabs(hd_FR_LA) > 0.20)
    {
        hd_FR_LA = copysign(0.20, hd_FR_LA);
    } 
    
   // first_order_filter(&hd_FR_LA_fil, &hd_FR_LA, para_hd_FR, hd_FR_arrSize);
/////////////////////////////////////////////////////////////
    if (motiontime < 20000 || transition_rate_temp < 1){
        hd_FR_LA = (double) 0.0;
    }
    else{
        hd_FR_LA = hd_FR_LA;
    }


  return hd_FR_LA;
}