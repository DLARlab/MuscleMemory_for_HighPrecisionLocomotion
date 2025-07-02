#ifndef IMUESTIMATOR_H
#define IMUESTIMATOR_H

void imu_estimator(float (&imu_quaternion)[4], float (&trunk_YPR_fil)[3], float (&trunk_YPR)[3], float (&imu_acceleration)[3], float (&trunk_Vel_fil)[3], float (&imu_acceleration_fil)[3], float (&trunk_Vel_strongfil)[3], float para_imu, float para_imu_strong);


#endif
