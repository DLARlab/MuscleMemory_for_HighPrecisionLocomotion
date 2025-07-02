int GroundTouchPronking(Eigen::Matrix<float,3,4> Js_fForce, int Foot_force[]);
void LeftRightDetech(int sin_count, int (&Foot_force)[4], int& trigger_forceF, int& trigger_forceR, int (&trigger_R_L)[4],  long motiontime, float PD_time);
void FirstlastDetect(int (&flag_st)[4], int (&flag_fly)[4], int (&Foot_force)[4], int& trigger_forceF, int& trigger_forceR, int (&trigger_R_L)[4], long motiontime, float PD_time);
