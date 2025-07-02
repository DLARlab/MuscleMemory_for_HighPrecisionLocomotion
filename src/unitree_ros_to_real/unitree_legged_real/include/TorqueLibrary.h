#ifndef TORQUELIBRARY_H
#define TORQUELIBRARY_H

void torque_record(float* Torque_Profile_st, float (*torque)[12], float (*Error)[12], float (*dError)[12], float (*Pre_Error)[12], float (*Pre_dError)[12], float (*Feedforward_Error_tor)[12], float (*pre_torque_fil_zero)[12], float (*pre_torque_fil_err_tor)[12], float (*total_torque_fil)[12], float (*pre_torque_fil_total)[12], int rows, int stride_counter);

void torque_library(float* Torque_Profile_st, int size);

void Torqueprofile_interpolation(float (&Torque_Profile_st)[12][21][230],float (&ILC_torque)[12], std::vector<float> &x_velocity, float fly_sel_velocity, float (&desired_velocity_trunk)[3], int int_pre_number, long motiontime);

#endif
