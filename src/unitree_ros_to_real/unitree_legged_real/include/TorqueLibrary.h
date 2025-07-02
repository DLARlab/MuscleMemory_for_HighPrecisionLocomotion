#ifndef TORQUELIBRARY_H
#define TORQUELIBRARY_H

void torque_record(float (&torque_stance)[300][12], float (&Error_stance)[300][12], float (&dError_stance)[300][12], float (&Pre_Error_stance)[300][12], float (&Pre_dError_stance)[300][12], float (&Feedforward_Error_tor_stance)[300][12], float (&pre_torque_stance_fil_zero)[300][12], float (&pre_torque_stance_fil_err_tor)[300][12], float (&total_stance_torque_fil)[300][12], float (&pre_torque_stance_fil_total)[300][12], int stride_counter);

void torque_library(float (&Torque_Profile_st)[12][21][230]);

void Torqueprofile_interpolation(float (&Torque_Profile_st)[12][21][230],float (&ILC_torque)[12], std::vector<float> &x_velocity, float fly_sel_velocity, float (&desired_velocity_trunk)[3], int int_pre_number, long motiontime);

void loadFootForceData(float (&Footforce_con)[21][12][21]);
void loadDdxData(float (&ddx_con)[21][3][21]);
#endif
