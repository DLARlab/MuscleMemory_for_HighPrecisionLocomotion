
#include <Eigen/Dense>
#include <Eigen/Core>

#include <algorithm>
#include "TouchdownDetect.h"

int GroundTouchPronking(Eigen::Matrix<float,3,4> Js_fForce, int Foot_force[])
{
    int RearTouchState, FrontTouchState, GroundTouchState;
           
           //////detect touchdown events of feet

            if ((-Js_fForce(2,0) > 20 || Foot_force[0] > 50) && (-Js_fForce(2,1) > 20 || Foot_force[1] > 50)){
                FrontTouchState = 1;
            }
            else{
                FrontTouchState = 0;
            }

            if ((-Js_fForce(2,2) > 20 || Foot_force[2] > 50) && (-Js_fForce(2,3) > 20 || Foot_force[3] > 50)){
                RearTouchState = 1;
            }
            else{
                RearTouchState = 0;
            }

            if ((FrontTouchState == 1) && (RearTouchState ==1 )){
              GroundTouchState = 2;
            }
            else if((FrontTouchState == 1) && (RearTouchState ==0 )){
              GroundTouchState = 4;
            }
            else if((FrontTouchState == 0) && (RearTouchState ==1 )){
              GroundTouchState = 6;
            }
            else if((FrontTouchState == 0) && (RearTouchState ==0 )){
              GroundTouchState = 8;
            }
            else{
              GroundTouchState = 0;
            }
    return GroundTouchState;
}

void LeftRightDetech(int sin_count, int (&Foot_force)[4], int& trigger_forceF, int& trigger_forceR, int (&trigger_R_L)[4], long motiontime, float PD_time)
{
  
////////////////////use different at different phase//////
  if ( motiontime < PD_time){

      if ( trigger_R_L[0] == 1){
          trigger_forceF = Foot_force[1];
          trigger_forceR = Foot_force[3];
          trigger_R_L[0] = 0;
          trigger_R_L[1] = 1;
      }
      else{
          trigger_forceF = Foot_force[1];
          trigger_forceR = Foot_force[3];
          trigger_R_L[0] = 1;
          trigger_R_L[1] = 0;
      }
  }
  else{
      if ( trigger_R_L[0] == 1){
          trigger_forceF = Foot_force[0];
          trigger_forceR = Foot_force[0];
          trigger_R_L[0] = 0;
          trigger_R_L[1] = 1;
      }
      else{
          trigger_forceF = Foot_force[0];
          trigger_forceR = Foot_force[0];
          trigger_R_L[0] = 1;
          trigger_R_L[1] = 0;
      }
  }
}
        
void FirstlastDetect(int (&flag_st)[4], int (&flag_fly)[4], int (&Foot_force)[4], int& trigger_forceF, int& trigger_forceR, int (&trigger_R_L)[4], long motiontime, float PD_time)
{
  float a, b;
    
      if (Foot_force[0] > 20){
        trigger_forceR = Foot_force[0];
      }
      else if (Foot_force[1] > 20){
        trigger_forceR = Foot_force[1];
      }
      else if (Foot_force[2] > 20){
        trigger_forceR = Foot_force[2];
      }
      else if (Foot_force[3] > 20){
        trigger_forceR = Foot_force[3];
      }
      else {
        trigger_forceR = Foot_force[0];
      }
    
      if ((Foot_force[0] < 0) && (Foot_force[1] < 0) && (Foot_force[2] < 0) && (Foot_force[3] < 0)){
        a = std::min(Foot_force[0], Foot_force[1]);
        b = std::min(Foot_force[2], Foot_force[3]);
        
        trigger_forceF =  std::min(a, b);
      }
      else {
        trigger_forceF = Foot_force[1];
      }
      
    
}