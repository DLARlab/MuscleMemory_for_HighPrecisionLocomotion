//////////IMU estimator/////////
#include <algorithm>
#include <cmath>
#include "LowpassFilter.h"

#include "ImuEstimator.h"
//row
void imu_estimator(float (&imu_quaternion)[4], float (&trunk_YPR_fil)[3], float (&trunk_YPR)[3], float (&imu_acceleration)[3], float (&trunk_Vel_fil)[3], float (&imu_acceleration_fil)[3], float (&trunk_Vel_strongfil)[3], float para_imu, float para_imu_strong)
{
    
    float sinr, cosr, sinp, siny, cosy, roll, pitch, yaw;
    int imu_arrSize;

    imu_arrSize = sizeof(imu_acceleration_fil)/sizeof(imu_acceleration_fil[0]);

    sinr = 2 * (imu_quaternion[0] * imu_quaternion[1] + imu_quaternion[2] * imu_quaternion[3]);
    cosr = 1 - 2 * (pow(imu_quaternion[1],2) + pow(imu_quaternion[2], 2));
    roll = atan2 (sinr,cosr);
//pitch
    sinp = 2 * (imu_quaternion[0] * imu_quaternion[2] - imu_quaternion[3] * imu_quaternion[1]);

        if (fabs(sinp) >=1)
              {
                  pitch = copysign(M_PI_2, sinp);
              }
             else
              {
                 pitch = asin(sinp);
               }
           //yall
            siny = 2 * (imu_quaternion[0] * imu_quaternion[3] + imu_quaternion[1] * imu_quaternion[3]);
            cosy = 1 - 2 * (pow(imu_quaternion[1], 2) + pow(imu_quaternion[2], 2));
            yaw = atan2 (siny,cosy);

           trunk_YPR[0] = yaw;
           trunk_YPR[1] = pitch;
           trunk_YPR[2] = roll;
        
        first_order_filter(imu_acceleration_fil, imu_acceleration, para_imu, imu_arrSize);
        first_order_filter(trunk_YPR_fil, trunk_YPR, para_imu_strong, imu_arrSize);

        for(int i_v=0; i_v<3; i_v++){
            trunk_Vel_fil[i_v] = trunk_Vel_fil[i_v]+0.001*imu_acceleration_fil[i_v];        
        }

        
        first_order_filter(trunk_Vel_strongfil, trunk_Vel_fil, para_imu_strong, imu_arrSize);            
}