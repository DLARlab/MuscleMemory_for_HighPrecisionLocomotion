#ifndef GAITLIBRARY_H
#define GAITLIBRARY_H

void gait_library(float (&bez_params_st)[21][72], float (&bez_params_fly)[21][72], float (&duration_times_st)[21], float (&duration_times_fly)[21]);

void gaitpara_interpolation(Eigen::Matrix<float, 12, 6>& coeff_st, Eigen::Matrix<float, 12, 6>& coeff_fly, float& step_time_st, float& step_time_fly, float (&bez_params_st)[21][72], float (&bez_params_fly)[21][72], float (&duration_times_st)[21], float (&duration_times_fly)[21], std::vector<float> &x_velocity, float fly_sel_velocity, float (&desired_velocity_trunk)[3], long motiontime);

void gait_control_gains(float (&joint_Kp)[12], float (&joint_Kv)[12], float (&joint_Kp_f)[12], float (&joint_Kv_f)[12], float (&joint_Kp_fd)[12], float (&joint_Kv_fd)[12], int (&flag_st)[4], int (&flag_fly)[4], float (&t_norminal_st)[4], float (&t_norminal_fly)[4], float PD_time, float transition_rate_temp_st, long motiontime);


#endif
