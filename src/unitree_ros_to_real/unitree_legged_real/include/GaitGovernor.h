#ifndef GAITGOVERNOR_H
#define GAITGOVERNOR_H

float velocity_regulation(float& fly_sel_velocity, float& transition_rate_temp, float (&desired_velocity_trunk)[3], float (&trunk_Vel_strongfil)[3], long motiontime);

#endif