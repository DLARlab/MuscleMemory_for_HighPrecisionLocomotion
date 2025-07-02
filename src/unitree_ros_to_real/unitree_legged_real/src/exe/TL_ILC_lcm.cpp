/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>


#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

#include "a1_sgn.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <math.h>
#include <fstream>

using namespace Eigen;
using namespace std;

# include "bezier.h"
# include "clamp.h"
# include "TouchdownDetect.h"
# include "LowpassFilter.h"
# include "Interpolation.h"
# include "GaitLibrary.h"
# include "TorqueLibrary.h"
# include "GaitGovernor.h"
# include "pdflib.hpp"
# include "rnglib.hpp"
# include "ImuEstimator.h"
# include "textfileparser.h"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

ros::Publisher desired_pub;
ros::Publisher lowCmd_pub;
ros::Publisher Logical_pub;
ros::Publisher Imu_pub;
ros::Subscriber sub_joy_msg;
ros::Publisher lowState_pub; //for rviz visualization


template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(1000);
    }
}

class JoyListener
{
    public:
    float stride_vel;
    int Button_0;
    int Button_3;
    void JoyData_Callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

};

void JoyListener::JoyData_Callback(const sensor_msgs::Joy::ConstPtr &joy_msg){
    //left axis up and down
    stride_vel = joy_msg->axes[1];
    Button_0 = joy_msg->buttons[0];
    Button_3 = joy_msg->buttons[3];
    ROS_INFO("receive %.2f", stride_vel);
    ROS_INFO("receive %d", Button_0);
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(1000);

    long motiontime = 0;
    int rate_count = 0;
    int rate_count2 = 0;
    int rate_count3 = 0;
    int sin_count = 0;
    float qInit[12]={0};
    float qDes[12]={0};

    float Kp[12] = {0}; 
    float Kd[12] = {0};
    float torque[12] = {0}; 

    ////////////////variable zone/////////////////////////
    float standt,standtpre, step_time_stouch, step_time_fly, step_time_st,step_ct_fly,step_ct_stouch,step_ct_st, Jump_S,Jump_ini, para, para_imu, para_imu_strong, torque_para, st_switch,fly_switch,st_period, fly_period, Forw_tor, PD_time;
    float t_norminal_st[4], t_norminal_fly[4], t_norminal[4], t_norminal_sfast[4],t_pre[4],Store[4],Foot_force_fil[4],sen_Foot_force[4],sen_imu_acceleration[3], ILC_Switch[4],velocity_rate[4];
    int RearTouchState, FrontTouchState, GroundTouchState;
    int flag_st[4], flag_fly[4], flag_ai, flag_ai0, Force_arrSize, imu_arrSize,torque_arrSize, hd_FR_arrSize, stride_counter, transition_counter;
    int Foot_force[4], Foot_forcex[4], Foot_forcey[4], Foot_forcez[4],joy_butt[4], trigger_R_L[4];
    int trigger_forceF = 0, trigger_forceR = 0;
    float trunk_Vel[3],trunk_Vel_fil[3],trunk_Vel_strongfil[3], desired_velocity_trunk[3], imu_quaternion[4],angular_Vel[3], imu_acceleration[3],imu_acceleration_fil[3], hd_FR_LA_fil[3];
    float des_trunk_Vel[3];
    float sinr, cosr, sinp, siny, cosy, roll, pitch, yaw;
    float param_hip_Kp, param_hip_Kd, param_thigh_Kp, param_thigh_Kd, param_calf_Kp, param_calf_Kd;
    
    float All_joint[12] = {0},twist_trunk[6],All_joint_velfil[12], All_joint_vel[12], trunk_YPR[3], trunk_YPR_fil[3], imu_angular_vel[3], imu_Gyroscope[3];

    Matrix<float,18,1> q_base_joints;  Matrix<float,18,1> q_base_joints_ref;
    Matrix<float,12,1> All_ref_Joint; 
  
    Matrix<float,3,1> position_FRH; Matrix<float,3,1> position_FRT;Matrix<float,3,1> position_FRC;Matrix<float,3,1> position_FRF;Matrix<float,3,1> position_FLH; Matrix<float,3,1> position_FLT;Matrix<float,3,1> position_FLC;Matrix<float,3,1> position_FLF;
    Matrix<float,3,1> position_RRH; Matrix<float,3,1> position_RRT;Matrix<float,3,1> position_RRC;Matrix<float,3,1> position_RRF;Matrix<float,3,1> position_RLH; Matrix<float,3,1> position_RLT;Matrix<float,3,1> position_RLC;Matrix<float,3,1> position_RLF;

    Matrix<float,3,18> Jp_FRF, Jp_FLF, Jp_RRF, Jp_RLF;
    Matrix<float,3,18> Jp_BasePX, Jp_BasePY, Jp_BasePZ, Jp_BaseRZ;
    Matrix<float,3,3> Js_FRF, Js_FLF, Js_RRF, Js_RLF;
    Matrix<float,3,3> JsT_FRF, JsT_FLF, JsT_RRF, JsT_RLF;
    Matrix<float,3,1> position_BSZ;
    // reference position
    Matrix<float,3,1> position_ref_FRH; Matrix<float,3,1> position_ref_FRT;Matrix<float,3,1> position_ref_FRC;Matrix<float,3,1> position_ref_FRF;Matrix<float,3,1> position_ref_FLH; Matrix<float,3,1> position_ref_FLT;Matrix<float,3,1> position_ref_FLC;Matrix<float,3,1> position_ref_FLF;
    Matrix<float,3,1> position_ref_RRH; Matrix<float,3,1> position_ref_RRT;Matrix<float,3,1> position_ref_RRC;Matrix<float,3,1> position_ref_RRF;Matrix<float,3,1> position_ref_RLH; Matrix<float,3,1> position_ref_RLT;Matrix<float,3,1> position_ref_RLC;Matrix<float,3,1> position_ref_RLF;

    float lengh_xy_FRF,lengh_x_FR;
    float theta_FR,theta_FR_test, FR_LA, hd_FR_LA[3];
    float lengh_z_FR,lengh_z_FL, lengh_z_RR, lengh_z_RL;
    float L_leg_FR, L_leg_FL, L_leg_RR, L_leg_RL;
    float Kp_theta, Kd_theta, Kp_pitch, Kd_pitch, apha,beta, gama,vel_para, para_hd_FR, touchdelay, normaltime,tor_weight;
    float sel_velocity, fly_sel_velocity, St_sel_velocity, ini_sel_velocity,des_sel_velocity;
    float difference_sel_vel, difference_vel_realtime, difference_selv_integral;

    float ConsOffset = (0.305);

    float sin_mid_q[12] = {0.000f, 1.057f - ConsOffset, -2.200f, 0.000f, 1.057f  - ConsOffset, -2.200f, 0.000f, 1.057f - ConsOffset, -2.200f, 0.000f, 1.057f - ConsOffset, -2.200f};
    int flag_ai_d=1;
    int* p_flag_ai= &flag_ai_d;
    float pre_number = 0.0;
    int int_pre_number =  0;       //NUMBER of data of torque in previous step
    Matrix<float,6,1> coeff_st_FRH, coeff_st_FRT,  coeff_st_FLH, coeff_st_FLT, coeff_st_RRH, coeff_st_RRT,coeff_st_RLH, coeff_st_RLT;
    Matrix<float,6,1> coeff_st_FRC, coeff_st_FLC, coeff_st_RRC, coeff_st_RLC;

    //fly_phase

    Matrix<float,6,1> coeff_fly_FRH, coeff_fly_FRT, coeff_fly_FRC, coeff_fly_FLH, coeff_fly_FLT, coeff_fly_FLC, coeff_fly_RRH, coeff_fly_RRT, coeff_fly_RRC, coeff_fly_RLH, coeff_fly_RLT, coeff_fly_RLC;
    Matrix<float,6,1> coeff_stouch_FRH, coeff_stouch_FRT, coeff_stouch_FRC, coeff_stouch_FLH, coeff_stouch_FLT, coeff_stouch_FLC, coeff_stouch_RRH, coeff_stouch_RRT, coeff_stouch_RRC, coeff_stouch_RLH, coeff_stouch_RLT, coeff_stouch_RLC; 
    ////define the vector of bezier paremeters
    Matrix<float,12,6> coeff_st, coeff_fly, coeff_stouch;

    ///define the matrix for forward kinematic
    Matrix<float,4,4> H_FRF, H_FRH, H_FRT, H_FRC;
    Matrix<float,3,4>vector_joint_tao;
    Matrix<float,3,4> Js_fForce;

    float t_now;
    //
    float dq_joint[12] = {0};
    //
    float delta_q_FRH, delta_q_FRT, delta_q_FRC, delta_q_FLH, delta_q_FLT, delta_q_FLC, delta_q_RRH, delta_q_RRT, delta_q_RRC, delta_q_RLH, delta_q_RLT, delta_q_RLC, transition_rate, transition_rate_temp, transition_rate_temp_st;
    float Error_q[12],st_sel_Error_q[12], Error_dq[12], q_joint[12] = {0}, per_st_motor_torque[12], per_st_motor_torque_sat[12], per_st_motor_torque_fil[12],per_st_motor_fil_err_tor[12], per_st_motor_total_fil[12],per_st_motor_torque_fil_unshift[12],per_st_Error[12],per_st_dError[12];
    float per_stouch_motor_torque[12], per_stouch_motor_torque_sat[12], per_stouch_motor_torque_fil[12],per_stouch_motor_fil_err_tor[12], per_stouch_motor_total_fil[12],per_stouch_motor_torque_fil_unshift[12],per_stouch_Error[12],per_stouch_dError[12];

    float Store_q_FRH, Store_q_FRT, Store_q_FRC, Store_q_FLH, Store_q_FLT, Store_q_FLC, Store_q_RRH, Store_q_RRT, Store_q_RRC, Store_q_RLH, Store_q_RLT, Store_q_RLC;
    float fly_q[12], fly_dq[12];
    float St_q[12];
    float torque_q[12], Feedforward_Error_tor[12], Feedforward_Error_sat[12], ILC_torque[12];
    //
    float joint_angle_des[12], joint_Kp[12], joint_Kv[12], joint_tor_compensation[12],motor_torque[12],motor_torque_pre[12],original_torque[12], pre_bezier[8], joint_Kp_f[12], joint_Kv_f[12],joint_Kp_fd[12],joint_Kv_fd[12];
    float TorsoVelocity[6];
    float tao_thigh_pitch[4], Kp_tao_thigh_pitch[4],_pitchControlThighfil, _pitchControlThigh;//_pitchControlThigh is the thigh input getting from iteration policy for pitch control
    float _thinghMin = -2, _thinghMax = 2, _thingVMin = -4, _thinghVMax = 4;
    /////this is used for interpolate bezier parameter///////
    vector<float> x_velocity;
    
    float  torque_stance[439][12], Pre_torque_stance[439][12],Pre_Feedforward_Error_tor_stance[439][12],Pre_torque_stance_fil[439][12], pre_torque_stance_copy[12][439], pre_Feedforward_Error_tor_copy[12][439], pre_torque_stance_fil_zero[439][12], pre_torque_stance_fil_err_tor[439][12], pre_torque_stance_fil_total[439][12],total_stance_torque_fil[439][12], Error_stance[439][12],Feedforward_Error_tor_stance[439][12], dError_stance[439][12],Pre_Error_stance[439][12],Pre_dError_stance[439][12],pre_Feedforward_Error_tor[439][12];
    float  torque_stouch[519][12], Pre_torque_stouch[519][12],Pre_Feedforward_Error_tor_stouch[519][12],Pre_torque_stouch_fil[519][12], pre_torque_stouch_copy[12][519], pre_Feedforward_Error_tor_copy2[12][519], pre_torque_stouch_fil_zero[519][12], pre_torque_stouch_fil_err_tor[519][12], pre_torque_stouch_fil_total[519][12],total_stouch_torque_fil[519][12], Error_stouch[519][12],Feedforward_Error_tor_stouch[519][12], dError_stouch[519][12],Pre_Error_stouch[519][12],Pre_dError_stouch[519][12],pre_Feedforward_Error_tor2[519][12];

    
    Matrix<float,20,1> sf_b;
    Matrix<float,20,1> sf_b_f;
    Matrix<float,20,1> sf_b_tor, sf_b_com;
    float s_fast, s_fast_com;

////////////////parameter setting area///////////////////////////////////////////////////////////////
    Jump_ini = 0;
    standt=(float)6000;
    standtpre=(float)3000;
    step_time_st=(float)0.2;
    
    step_time_fly=(float)0.1936;//0.1811;//0.15;////0.270163866360342;
    step_time_stouch=(float)0.5;
    PD_time = 10000;
    
    beta=0;
    para = (float) 0.03;
    torque_para = (float) 0.1;
    para_imu = (float) 0.1;
    para_imu_strong = (float) 0.0003;
    fly_switch = 0.8f;
    st_switch = 0.8f;
    st_period = 1.5f;
    fly_period = 1.5f;
    Forw_tor = 20.0f;
    touchdelay = 0.0f;
    normaltime = 1.0f;
    sel_velocity = 0.0f;
    fly_sel_velocity = 0.0f;
    St_sel_velocity = 0.0f;
   
    des_sel_velocity = 0.0f;
    stride_counter = 0;
    transition_counter = 0;
    
    transition_rate = 0.0f;
    transition_rate_temp = 0.0f;
    transition_rate_temp_st = 0.0f;
    difference_selv_integral = 0.0f;

    //////////initialize value here for later
    for (int i_a =0 ; i_a<21; i_a++){
         float a_velocity = (i_a * 1.0 -10.0)/10.0;  
         x_velocity.push_back(a_velocity);
    }

    for (int i_st = 0 ; i_st < 439 ; i_st++){
      for ( int j_st = 0 ; j_st<12; j_st++){
         torque_stance[i_st][j_st] = 0.0f;
         Pre_torque_stance[i_st][j_st] = 0.0f;
         Pre_torque_stance_fil[i_st][j_st] = 0.0f;
         

         pre_torque_stance_fil_zero[i_st][j_st] = 0.0f;  //only need to chage the the way of define. it doesn't inference the  what you did later
         pre_torque_stance_fil_err_tor[i_st][j_st] = 0.0f;

         Error_stance[i_st][j_st] = 0.0f;
         dError_stance[i_st][j_st] = 0.0f;
         Feedforward_Error_tor_stance[i_st][j_st] = 0.0f;
        

         Pre_Error_stance[i_st][j_st] = 0.0f;
         Pre_dError_stance[i_st][j_st] = 0.0f;
         Pre_Feedforward_Error_tor_stance[i_st][j_st] = 0.0f;

         pre_torque_stance_copy[j_st][i_st] = 0.0f;
         pre_Feedforward_Error_tor[i_st][j_st] = 0.0f;
         pre_Feedforward_Error_tor_copy[j_st][i_st] = 0.0f;

         total_stance_torque_fil[i_st][j_st] = 0.0f;
         pre_torque_stance_fil_total[i_st][j_st] = 0.0f;

        } 
     } 

    for (int i_st = 0 ; i_st < 519 ; i_st++){
      for ( int j_st = 0 ; j_st<12; j_st++){
         torque_stouch[i_st][j_st] = 0.0f;
         Pre_torque_stouch[i_st][j_st] = 0.0f;
         Pre_torque_stouch_fil[i_st][j_st] = 0.0f;
         

         pre_torque_stouch_fil_zero[i_st][j_st] = 0.0f;  //only need to chage the the way of define. it doesn't inference the  what you did later
         pre_torque_stouch_fil_err_tor[i_st][j_st] = 0.0f;

         Error_stouch[i_st][j_st] = 0.0f;
         dError_stouch[i_st][j_st] = 0.0f;
         Feedforward_Error_tor_stouch[i_st][j_st] = 0.0f;
        

         Pre_Error_stouch[i_st][j_st] = 0.0f;
         Pre_dError_stouch[i_st][j_st] = 0.0f;
         Pre_Feedforward_Error_tor_stouch[i_st][j_st] = 0.0f;

         pre_torque_stouch_copy[j_st][i_st] = 0.0f;
         pre_Feedforward_Error_tor2[i_st][j_st] = 0.0f;
         pre_Feedforward_Error_tor_copy2[j_st][i_st] = 0.0f;

         total_stouch_torque_fil[i_st][j_st] = 0.0f;
         pre_torque_stouch_fil_total[i_st][j_st] = 0.0f;

        } 
     } 

    for(int i_ini=0; i_ini<4; i_ini++){     
      flag_st[i_ini]=(int)0;
      flag_fly[i_ini]=(int)0;
      ILC_Switch[i_ini] = (float) 1;
      t_norminal_st[i_ini] = 0.0;
      t_norminal_fly[i_ini] = 0.0;
      t_norminal[i_ini] = 0.0;
      t_norminal_sfast[i_ini] = 0.0;
      t_pre[i_ini] = 0.0;
      Foot_force_fil[i_ini] = 0.0;
      velocity_rate[i_ini] =(float) 1.0;
      Kp_tao_thigh_pitch[i_ini] = (float) -0.0;///-60.0;
      tao_thigh_pitch[i_ini] = (float) 0.0; 
      _pitchControlThighfil = (float) 0.0;
    } 

    for(int i_ini=0; i_ini<4; i_ini++){     
     Foot_force[i_ini] = 0;
     Foot_forcex[i_ini] = 0;
     Foot_forcey[i_ini] = 0;
     Foot_forcez[i_ini] = 0;
     joy_butt[i_ini] = 0;
     trigger_R_L[i_ini] = 0;
    }
    for(int i_ini=0; i_ini<6; i_ini++){     
     TorsoVelocity[i_ini] = 0;
    }       
    for(int i_ini = 0; i_ini < 12; i_ini++){     
      joint_angle_des[i_ini] = 0.0;
      joint_tor_compensation[i_ini] = 0.0;
      All_joint_velfil[i_ini] = 0.0;

      motor_torque[i_ini] = 0.0;
      motor_torque_pre[i_ini] = 0.0;
      original_torque[i_ini] = 0.0;
      per_st_motor_torque_fil_unshift[i_ini] = 0.0;
      per_st_motor_torque_fil[i_ini] = 0.0;
      per_st_motor_total_fil[i_ini] = 0.0;
      per_st_motor_fil_err_tor[i_ini] = 0.0;

      torque_q[i_ini] = 0.0;
      Feedforward_Error_tor[i_ini] = 0.0;
      Feedforward_Error_sat[i_ini] = 0.0;
      ILC_torque[i_ini] = 0.0;

      per_st_Error[i_ini] = 0.0;
      per_st_dError[i_ini] = 0.0;

      st_sel_Error_q[i_ini] = 0.12; //////to avoid wrong offset at beginning

      fly_q[i_ini] = 0.0;
      fly_dq[i_ini] = 0.0;      
    } 


        for(int i_vel=0; i_vel<3; i_vel++){
        trunk_Vel[i_vel] = 0;
        des_trunk_Vel[i_vel] = 0.0;
        trunk_Vel_fil[i_vel] = 0.0;
        trunk_Vel_strongfil[i_vel] = 0.0;
        imu_acceleration_fil[i_vel] = 0.0;
        desired_velocity_trunk[i_vel] = 0.0;
        hd_FR_LA_fil[i_vel] = 0.0;
        hd_FR_LA[i_vel] = 0.0;

        trunk_YPR[i_vel] = 0.0;
        trunk_YPR_fil[i_vel] = 0.0;
        imu_angular_vel[i_vel]= 0.0; 
        imu_Gyroscope[i_vel] = 0.0;
              } 

        Force_arrSize = sizeof(Foot_force_fil)/sizeof(Foot_force_fil[0]);
        imu_arrSize = sizeof(imu_acceleration_fil)/sizeof(imu_acceleration_fil[0]);
        torque_arrSize = sizeof(per_st_motor_torque_fil)/sizeof(per_st_motor_torque_fil[0]);
        int velfil_num = sizeof(All_joint_velfil)/sizeof(All_joint_velfil[0]);
        vel_para = 0.003f;

        hd_FR_arrSize = sizeof(hd_FR_LA_fil)/sizeof(hd_FR_LA_fil[0]);
        para_hd_FR = (float) 0.003;
        
        ///////////////////////////////////////////////////////////////
        ros::NodeHandle nm;
        JoyListener joy_listener;
        sub_joy_msg = nm.subscribe("/joy", 100, &JoyListener::JoyData_Callback, &joy_listener);


        //////////////////
        //////////get parameters from the library/////////
        float bez_params_st[21][72] = { 0.0 };//define a place to store
        float bez_params_fly[21][72] = { 0.0 };//define a place to store
        float bez_params_stouch[21][72] = { 0.0 };//define a place to store

        float duration_times_st[21] = { 0.0 };//define a place to store
        float duration_times_fly[21] = { 0.0 };//define a place to store
        float duration_times_stouch[21] = { 0.0 };//define a place to store
        gait_library(bez_params_st, bez_params_fly, bez_params_stouch, duration_times_st, duration_times_fly, duration_times_stouch);

        //////////////////
        //////////get parameters from the torque library/////////
        float Torque_Profile_st[12][21][439] = { 0.0 };//define a place to store toruqe
        float Torque_Profile_stouch[12][21][519] = { 0.0 };//define a place to store toruqe

        torque_library(&Torque_Profile_st[0][0][0],439);
        torque_library(&Torque_Profile_stouch[0][0][0],519);
                            // // //debugging tool//Print the 2D vector
        // for (int i = 0; i < 12; i++) {
        //     for (int j = 0; j < 439; j++) {
        //         std::cout << Torque_Profile_st[0][i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
/////////////////////get parameters from the policy tablet////////////////////////
        std::string filename = "/home/dlar/test/policy_values_5.txt";
        std::vector<std::vector<float>> floatVector = readTxtFileTo2DVector(filename);

        if (floatVector.empty()) {
            std::cerr << "Error: Failed to read the file or the file is empty." << std::endl;
            return 1;
        }

        // Set precision to 1 decimal place
        std::cout << std::fixed << std::setprecision(1);

        // Access the values in the 2D vector and display them
        for (const auto& row : floatVector) {
            for (const float& value : row) {
              std::cout << value << " ";
            }
            std::cout << "\n";
        }
            
///////////////////////initialize joints after stand/////////
        for (int i_bez = 0; i_bez < 3; i_bez++){
	        sin_mid_q[i_bez] = bez_params_st[0][i_bez];
            sin_mid_q[i_bez+3] = bez_params_st[0][i_bez];
            sin_mid_q[i_bez+6] = bez_params_st[0][i_bez];
            sin_mid_q[i_bez+9] = bez_params_st[0][i_bez];
        }
        for (int i_leg = 0; i_leg < 3; i_leg++){
	        sin_mid_q[i_leg * 3 + 1 ] = bez_params_st[0][i_leg * 3 +1] - ConsOffset;

        }


///////////////////////////////////////////////////////////

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    std_msgs::Int8 jump_switch_msg;
    std_msgs::Float32MultiArray desired_q_msg;
    
    //declare a memery space for data in desired(add actual value after filter in the end)
    desired_q_msg.data.resize(45);     

    UNITREE_LEGGED_SDK::Safety safe(LeggedType::A1);

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;        // 禁止位置环
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;        // 禁止速度环
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    while (ros::ok()){
        ros::Time begin = ros::Time::now();

        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        // printf("FR_2 position: %f\n",  RecvLowROS.motorState[FR_2].q);
        
        for(int i_leg = 0; i_leg < 4; i_leg++){
            Foot_force[i_leg] = RecvLowROS.footForce[i_leg];
            imu_quaternion[i_leg] = RecvLowROS.imu.quaternion[i_leg];
        }
        for(int i_v=0; i_v<3; i_v++){
            imu_acceleration[i_v]=RecvLowROS.imu.accelerometer[i_v];
            trunk_Vel[i_v]=trunk_Vel[i_v]+0.001*RecvLowROS.imu.accelerometer[i_v]; 

            imu_Gyroscope[i_v]=RecvLowROS.imu.gyroscope[i_v];
        } 

            // for(int i_joint = 0; i_joint < 100; i_joint++){
            //     std::cerr<<"Torque_Profile_st    "<<Torque_Profile_st[12][15][i_joint]<< std::endl;
            // }
        // Foot_force[0] = RecvLowROS.footForce[0];
        
        GroundTouchState = GroundTouchPronking(Js_fForce, Foot_force); 
        
        // lowState_realtime.publish(RecvLowROS.tick);
            Imu_pub.publish(RecvLowROS.imu);
            lowState_pub.publish(RecvLowROS);
            lowCmd_pub.publish(SendLowROS);
            jump_switch_msg.data = flag_st[0];
            Logical_pub.publish(jump_switch_msg);                        
            ////publish desired trajectories////////
            desired_q_msg.data.clear();

            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(q_joint[i_joint]);
            }
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(dq_joint[i_joint]);
            }
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(All_joint_velfil[i_joint]);
            }
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(per_st_motor_total_fil[i_joint]);
            }
            for(int i_leg = 0; i_leg < 4; i_leg++){
                desired_q_msg.data.push_back(t_norminal_st[i_leg]);;
            }
            //desired_q_msg.data.push_back(t_norminal_st[0]);
            desired_q_msg.data.push_back(GroundTouchState);
            // desired_q_msg.data.push_back(per_st_motor_total_fil[2]);
            // desired_q_msg.data.push_back(per_st_motor_total_fil[8]);
            desired_q_msg.data.push_back(trunk_Vel[0]);
            desired_q_msg.data.push_back(trunk_Vel_fil[0]);
            desired_q_msg.data.push_back(trunk_Vel_strongfil[0]);
            desired_q_msg.data.push_back(fly_sel_velocity);
            desired_q_msg.data.push_back(hd_FR_LA[0]);
            desired_q_msg.data.push_back(hd_FR_LA_fil[0]);
            desired_q_msg.data.push_back(tao_thigh_pitch[0]);
            desired_q_msg.data.push_back(trunk_YPR_fil[1]);
   
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(ILC_torque[i_joint]);
            }
            
            //////////////record the data after the zero phase filter////////////////
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(per_st_motor_torque_fil[i_joint]);
            }

            //////////////torque generated by feedforword error////////////////
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(Feedforward_Error_sat[i_joint]);
            }

            //////////////error infromation////////////////
            for(int i_joint = 0; i_joint < 12; i_joint++){
                desired_q_msg.data.push_back(Error_q[i_joint]);
            }
            // for(int i_leg = 0; i_leg < 4; i_leg++){
            //     desired_q_msg.data.push_back(flag_st[i_leg]);
            // }

            ///////////////////////////////////////////////
            desired_pub.publish(desired_q_msg);

            ////collect gait params////
            gaitpara_interpolation(coeff_st, coeff_fly,coeff_stouch, step_time_st, step_time_fly, step_time_stouch, bez_params_st, bez_params_fly, bez_params_stouch, duration_times_st, duration_times_fly, duration_times_stouch, x_velocity, fly_sel_velocity, desired_velocity_trunk, motiontime);
           // Torqueprofile_interpolation(Torque_Profile_st,ILC_torque, x_velocity, fly_sel_velocity, desired_velocity_trunk, int_pre_number, motiontime);

        if (motiontime < PD_time || transition_rate_temp < 1){
          ConsOffset = (double) (0.305);
          Kd_theta =  2.0;
        }
        else{
          ConsOffset = (double) (0.305);

          if (transition_rate_temp_st > 1 )
          {
            transition_rate = 1;//transition_counter/8;
            // ConsOffset = 0.000 * (1 - transition_rate);
          }
          // else{
          //   transition_rate = 1;
          //   ConsOffset = 0;
          // }         
          Kd_theta =  2.0;            
        }

        

        if(initiated_flag == true){
            motiontime++;
            SendLowROS.motorCmd[FR_0].tau = -0.65f;
            SendLowROS.motorCmd[FL_0].tau = +0.65f;
            SendLowROS.motorCmd[RR_0].tau = -0.65f;
            SendLowROS.motorCmd[RL_0].tau = +0.65f;
         if( motiontime >= 0){

                // printf("%d\n", motiontime);
                // printf("%d %f %f %f\n", FR_0, RecvLowROS.motorState[FR_0].q, RecvLowROS.motorState[FR_1].q, RecvLowROS.motorState[FR_2].q);
                // printf("%f %f \n",  RecvLowROS.motorState[FR_0].mode, RecvLowROS.motorState[FR_1].mode);
                
                    // first, get record initial position
                    // if( motiontime >= 100 && motiontime < 500){
                    if( motiontime >= 0 && motiontime < 10){
                        qInit[0] = RecvLowROS.motorState[FR_0].q;
                        qInit[1] = RecvLowROS.motorState[FR_1].q;
                        qInit[2] = RecvLowROS.motorState[FR_2].q;
                        qInit[3] = RecvLowROS.motorState[FL_0].q;
                        qInit[4] = RecvLowROS.motorState[FL_1].q;
                        qInit[5] = RecvLowROS.motorState[FL_2].q;
                        qInit[6] = RecvLowROS.motorState[RR_0].q;
                        qInit[7] = RecvLowROS.motorState[RR_1].q;
                        qInit[8] = RecvLowROS.motorState[RR_2].q;
                        qInit[9] = RecvLowROS.motorState[RL_0].q;
                        qInit[10] = RecvLowROS.motorState[RL_1].q;
                        qInit[11] = RecvLowROS.motorState[RL_2].q;
                    }

                    for(int i_motor = 0; i_motor < 12; i_motor++){
                            All_joint[i_motor] = RecvLowROS.motorState[i_motor].q;
                            All_joint_vel[i_motor] = RecvLowROS.motorState[i_motor].dq;
                    } 
                    first_order_filter(All_joint_velfil, All_joint_vel, vel_para, velfil_num);

                    first_order_filter(imu_angular_vel, imu_Gyroscope, para_imu, imu_arrSize);
                    imu_estimator(imu_quaternion, trunk_YPR_fil, trunk_YPR, imu_acceleration, trunk_Vel_fil, imu_acceleration_fil, trunk_Vel_strongfil, para_imu, para_imu_strong);


                        ///////Torso's velocity which is estimated in saggital plane in stance phace/////
                    TorsoVelocity[0] = 0.2 * cos(All_joint[2] + All_joint[1]) * (All_joint_vel[2] + All_joint_vel[1]) + 0.2 * cos( All_joint[1]) * (All_joint_vel[1]) + 0.1;


                    joy_butt[0] = joy_butt[1];
                    joy_butt[1] = joy_listener.Button_0;

                    joy_butt[2] = joy_butt[3];
                    joy_butt[3] = joy_listener.Button_3;

                    // if( joy_butt[0] == 0 && joy_butt[1] ==1){
                    //     Kp_tao_thigh_pitch = Kp_tao_thigh_pitch -1.0f;
                    //         if (Kp_tao_thigh_pitch < 1.0f){
                    //             Kp_tao_thigh_pitch = 1.0f;
                    //         }                                
                    // } 

                    // if( joy_butt[2] == 0 && joy_butt[3] ==1){
                    //     Kp_tao_thigh_pitch = Kp_tao_thigh_pitch +1.0f;
                    //         if (Kp_tao_thigh_pitch > 10.0f){
                    //             Kp_tao_thigh_pitch = 10.0f;
                    //         }
                    // }
                    // step_ct_st = (float)(1/step_time_st) * (1/Kp_tao_thigh_pitch);
                    // step_ct_fly = (float)(1/step_time_fly) * (1/Kp_tao_thigh_pitch);

                    step_ct_st = (float)(1/step_time_st);
                    step_ct_fly = (float)(1/step_time_fly);
                    step_ct_stouch = (float)(1/step_time_stouch);


                    if( motiontime >= 10 && motiontime < 3000){
                        // printf("%f %f %f\n", );
                        rate_count++;
                        double rate = rate_count/2800.0;                       // needs count to 1500

                        if((param_hip_Kp) > 20 && (param_hip_Kd) > 1  && (param_thigh_Kp) > 20  && (param_thigh_Kd) > 2 && (param_calf_Kp) > 30  && (param_calf_Kd) > 2 && (param_hip_Kp) < 300 && (param_hip_Kd) < 50  && (param_thigh_Kp) < 300  && (param_thigh_Kd) < 50 && (param_calf_Kp) < 300  && (param_calf_Kd) < 50)
                        {
                            for(int i_motor = 0; i_motor < 4; i_motor++){
                                    Kp[0 + 3 * i_motor] = param_hip_Kp;
                                    Kp[1 + 3 * i_motor] = param_thigh_Kp;
                                    Kp[2 + 3 * i_motor] = param_calf_Kp;

                                    Kd[0 + 3 * i_motor] = param_hip_Kd;
                                    Kd[1 + 3 * i_motor] = param_thigh_Kd;
                                    Kd[2 + 3 * i_motor] = param_calf_Kd;
                                }
                        }
                        else{
                            for(int i_motor = 0; i_motor < 4; i_motor++){
                                    Kp[0 + 3 * i_motor] = 35.0f;
                                    Kp[1 + 3 * i_motor] = 90.0f;
                                    Kp[2 + 3 * i_motor] = 130.0f;

                                    Kd[0 + 3 * i_motor] = 3.0f;
                                    Kd[1 + 3 * i_motor] = 3.0f;
                                    Kd[2 + 3 * i_motor] = 5.0f;
                                }
                        }


                        for(int i_motor = 0; i_motor < 12; i_motor++){
                            qDes[i_motor] = jointLinearInterpolation(qInit[i_motor], sin_mid_q[i_motor], rate);  
                            q_joint[i_motor] = qDes[i_motor];
                        }                   
                   

                        for(int i_leg = 0; i_leg < 4; i_leg++){
                            int check;
                            SendLowROS.motorCmd[0 + 3 * i_leg].q = qDes[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].dq = 0;
                            SendLowROS.motorCmd[0 + 3 * i_leg].Kp = Kp[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].Kd = Kd[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].tau = -0.65f * legsgn(i_leg%2);

                            SendLowROS.motorCmd[1 + 3 * i_leg].q = qDes[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].dq = 0;
                            SendLowROS.motorCmd[1 + 3 * i_leg].Kp = Kp[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].Kd = Kd[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].tau = 0.0f;

                            SendLowROS.motorCmd[2 + 3 * i_leg].q = qDes[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].dq = 0;
                            SendLowROS.motorCmd[2 + 3 * i_leg].Kp = Kp[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].Kd = Kd[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].tau = 0.0f;
                            std::cout << "Checking: the checking" << std::endl;
                        }
                    }


                if( motiontime >= 3000){
                    /////make it stand on the ground for a while for testing///
                    if( motiontime >= 3000 && motiontime < 5000){
                        // printf("%f %f %f\n", );

                        double rate = rate_count/2800.0;                       // needs count to 1500

                        if((param_hip_Kp) > 20 && (param_hip_Kd) > 1  && (param_thigh_Kp) > 20  && (param_thigh_Kd) > 2 && (param_calf_Kp) > 30  && (param_calf_Kd) > 2 && (param_hip_Kp) < 300 && (param_hip_Kd) < 50  && (param_thigh_Kp) < 300  && (param_thigh_Kd) < 50 && (param_calf_Kp) < 300  && (param_calf_Kd) < 50)
                        {
                            for(int i_motor = 0; i_motor < 4; i_motor++){
                                    Kp[0 + 3 * i_motor] = param_hip_Kp;
                                    Kp[1 + 3 * i_motor] = param_thigh_Kp;
                                    Kp[2 + 3 * i_motor] = param_calf_Kp;

                                    Kd[0 + 3 * i_motor] = param_hip_Kd;
                                    Kd[1 + 3 * i_motor] = param_thigh_Kd;
                                    Kd[2 + 3 * i_motor] = param_calf_Kd;
                                }
                        }
                        else{
                            for(int i_motor = 0; i_motor < 4; i_motor++){
                                    Kp[0 + 3 * i_motor] = 70.0f;
                                    Kp[1 + 3 * i_motor] = 100.0f;
                                    Kp[2 + 3 * i_motor] = 100.0f;

                                    Kd[0 + 3 * i_motor] = 5.0f;
                                    Kd[1 + 3 * i_motor] = 5.0f;
                                    Kd[2 + 3 * i_motor] = 5.0f;
                                }
                        }


                        for(int i_motor = 0; i_motor < 12; i_motor++){
                            q_joint[i_motor] = jointLinearInterpolation(qInit[i_motor], sin_mid_q[i_motor], rate);                         
                        }   

                        for(int j=0; j<12; j++){
                            Error_q[j] = q_joint[j] - All_joint[j];          
                        }

                        for(int e_j = 0; e_j < 12; e_j++) dq_joint[e_j] = 0;

                        // for(int e_j = 0; e_j < 12; e_j++){
                        //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                        // }

                        ///calculate toques for 12 joints
                        ///for ( int i_j = 0; i_j < 12; i_j++ ) torque[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];
                    }              

                    // float sin_joint[12];
                    // last, do sine wave
                    // this 5000 here related to some values in the subfunction
                    if( motiontime >= 5000){
                        sin_count++; 
                        Jump_S = sin_count/1000.0;
                        t_now=Jump_S;
                        //LeftRightDetech(sin_count, Foot_force, trigger_forceF, trigger_forceR, trigger_R_L, motiontime, PD_time);
                        FirstlastDetect(flag_st, flag_fly, Foot_force, trigger_forceF, trigger_forceR, trigger_R_L, motiontime, PD_time);
                        // for(int i_leg = 0; i_leg < 4; i_leg++){
                        //    sin_joint[2 + 3 * i_leg] = 0.2f * sin(1.0 * 2.0 * sin_count/1000.0);
                        //    sin_joint[1 + 3 * i_leg] = 0.1f * sin(0.5 * 2.0 * sin_count/1000.0);                       
                        // }  

                        if((param_hip_Kp) > 20 && (param_hip_Kd) > 1  && (param_thigh_Kp) > 20  && (param_thigh_Kd) > 2 && (param_calf_Kp) > 30  && (param_calf_Kd) > 2 && (param_hip_Kp) < 300 && (param_hip_Kd) < 50  && (param_thigh_Kp) < 300  && (param_thigh_Kd) < 50 && (param_calf_Kp) < 300  && (param_calf_Kd) < 50)
                            {
                                for(int i_leg = 0; i_leg < 4; i_leg++){
                                        Kp[0 + 3 * i_leg] = param_hip_Kp;
                                        Kp[1 + 3 * i_leg] = param_thigh_Kp;
                                        Kp[2 + 3 * i_leg] = param_calf_Kp;

                                        Kd[0 + 3 * i_leg] = param_hip_Kd;
                                        Kd[1 + 3 * i_leg] = param_thigh_Kd;
                                        Kd[2 + 3 * i_leg] = param_calf_Kd;
                                    }
                            }
                            else{
                                for(int i_leg = 0; i_leg < 4; i_leg++){
                                        joint_Kp[0 + 3 * i_leg] = 50.0f;
                                        joint_Kp[1 + 3 * i_leg] = 100.0f;
                                        joint_Kp[2 + 3 * i_leg] = 100.0f;

                                        joint_Kv[0 + 3 * i_leg] = 10.0f;
                                        joint_Kv[1 + 3 * i_leg] = 10.0f;
                                        joint_Kv[2 + 3 * i_leg] = 10.0f;

                                        joint_Kp_f[0 + 3 * i_leg] = 50.0f;
                                        joint_Kp_f[1 + 3 * i_leg] = 70.0f;
                                        joint_Kp_f[2 + 3 * i_leg] = 70.0f;

                                        joint_Kv_f[0 + 3 * i_leg] = 5.0f;
                                        joint_Kv_f[1 + 3 * i_leg] = 5.0f;
                                        joint_Kv_f[2 + 3 * i_leg] = 5.0f;

                                        joint_Kp_fd[0 + 3 * i_leg]= 0;
                                        joint_Kp_fd[1 + 3 * i_leg]= 0;
                                        joint_Kp_fd[2 + 3 * i_leg]= 0;

                                        joint_Kv_fd[0 + 3 * i_leg] = 0;
                                        joint_Kv_fd[1 + 3 * i_leg] = 0;
                                        joint_Kv_fd[2 + 3 * i_leg] = 0;
                                }
                            }

                        gait_control_gains(joint_Kp, joint_Kv, joint_Kp_f, joint_Kv_f, joint_Kp_fd, joint_Kv_fd, flag_st, flag_fly, t_norminal_st, t_norminal_fly, PD_time, transition_rate_temp_st, motiontime);
                                /////for front right leg


                                /////this is for the feedforward torque

                        for(int j=0; j<12; j++){
                            Error_q[j] = q_joint[j] - All_joint[j];          
                        }

                        // for(int e_j = 0; e_j < 12; e_j++) dq_joint[e_j] = 0;

                        for(int e_j = 0; e_j < 12; e_j++){
                            Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                        }
                        //////////set the tao_thigh_pitch  ////////////
                        _pitchControlThigh = dual_interpolate(floatVector, trunk_YPR_fil[1], imu_angular_vel[1], _thinghMin, _thinghMax, _thingVMin, _thinghVMax);
                         first_order_filter(&_pitchControlThighfil, &_pitchControlThigh, para_imu, Force_arrSize);
                        if (motiontime > PD_time + 3000){
                            if (GroundTouchState == 2){
                                for(int i_leg = 0; i_leg < 4; i_leg++){
                                    tao_thigh_pitch[i_leg] = Kp_tao_thigh_pitch[i_leg] * (trunk_YPR_fil[1]);
                                    // tao_thigh_pitch[i_leg] = 0.5 * _pitchControlThighfil;
                                }
                            }
                            if (GroundTouchState == 4){
                                for(int i_leg = 0; i_leg < 2; i_leg++){
                                    tao_thigh_pitch[i_leg] = Kp_tao_thigh_pitch[i_leg] * (trunk_YPR_fil[1]);
                                    // tao_thigh_pitch[i_leg] = 0.5 * _pitchControlThighfil;
                                }
                                for(int i_leg = 2; i_leg < 4; i_leg++){
                                    tao_thigh_pitch[i_leg] = 0.0f;
                                }
                            }
                            if (GroundTouchState == 6){
                                for(int i_leg = 0; i_leg < 2; i_leg++){
                                    tao_thigh_pitch[i_leg] = 0.0f;
                                }
                                for(int i_leg = 2; i_leg < 4; i_leg++){
                                    tao_thigh_pitch[i_leg] = Kp_tao_thigh_pitch[i_leg] * (trunk_YPR_fil[1]);
                                    // tao_thigh_pitch[i_leg] = 0.5 * _pitchControlThighfil;
                                }
                            }
                            else if (GroundTouchState == 8){
                                for(int i_leg = 0; i_leg < 4; i_leg++){
                                    tao_thigh_pitch[i_leg] = 0.0f;
                                }
                            }
                           
                        }
                        else{
                            for(int i_leg = 0; i_leg < 4; i_leg++){
                                    tao_thigh_pitch[i_leg] = 0.0f;
                                }
                        }

                        normaltime = 1 + touchdelay;

                        if (t_now < 0.005 )//
                        {
                                stride_counter++;

                                if (motiontime > PD_time && transition_rate_temp > 1 ){
                                    transition_rate_temp_st = 2.0;
                                }
                                else{
                                    transition_rate_temp_st =0.0;
                                }  

                                torque_record(&Torque_Profile_st[0][0][0], torque_stance, Error_stance, dError_stance, Pre_Error_stance, Pre_dError_stance, Feedforward_Error_tor_stance, pre_torque_stance_fil_zero, pre_torque_stance_fil_err_tor, total_stance_torque_fil, pre_torque_stance_fil_total,439, stride_counter);
                                torque_record(&Torque_Profile_stouch[0][0][0], torque_stouch, Error_stouch, dError_stouch, Pre_Error_stouch, Pre_dError_stouch, Feedforward_Error_tor_stouch, pre_torque_stouch_fil_zero, pre_torque_stouch_fil_err_tor, total_stouch_torque_fil, pre_torque_stouch_fil_total, 519,stride_counter);
                                
                                
                                t_norminal_st[0] = 0;
                                t_norminal_fly[0] = 0;
                                t_norminal[0] = 0;
                                flag_st[0]=1; 
                                flag_fly[0]=0; 
            
                        }
                        else if ( (t_norminal_st[0] > st_switch  &&  trigger_forceF <= 0 && t_now < 0.6) )
                        { 
                                if (motiontime < PD_time ){
                                    transition_rate_temp = 0.0;
                                }
                                else{
                                    transition_counter++;
                                    transition_rate_temp  = 2.0;
                                }

                                ///////////desired velocity from remote control/////////////
                                if (motiontime < 16000){
                                    desired_velocity_trunk[0] = 0.3;
                                }
                                // else if (motiontime < 19000) {
                                //     desired_velocity_trunk[0] = 0.6;
                                // }                                
                                // else if (motiontime < 22000) {
                                //     desired_velocity_trunk[0] = 0.7;
                                // }
                                // else if (motiontime < 25000) {
                                //     desired_velocity_trunk[0] = 0.75;
                                // }                                
                                // else if (motiontime < 28000) {
                                //     desired_velocity_trunk[0] = 0.8;
                                // }
                                // else if (motiontime < 34000) {
                                //     desired_velocity_trunk[0] = 0.8;
                                // }                                                                                                
                                else if (motiontime < 40000) {
                                    desired_velocity_trunk[0] = 0.3;
                                }                               
                                else {
                                    desired_velocity_trunk[0] = 0.3;
                                }
                                ////////////////////////////////////////////////////////////

                                t_norminal_fly[0] = 0;
                                t_norminal_st[0] = 0;
                                t_norminal[0] = 0;
                                flag_st[0]=0;  
                                flag_fly[0]=1; 
                        }
                        else if ( (t_norminal_fly[0] > fly_switch && trigger_forceR > 20))//
                        {

                                if (motiontime > 20000 && transition_rate_temp > 1 )
                                {
                                transition_rate_temp_st = 2.0;//transition_counter/8;
                                //   ConsOffset = 0.000 * (1 - transition_rate);
                                }
                                else{
                                transition_rate_temp_st =0.0;
                                //   ConsOffset = 0;
                                }   
                                int_pre_number = 0;
                                t_norminal_st[0] = 0;
                                t_norminal_fly[0] = 0;
                                t_norminal[0] = 0;
                                flag_st[0]=2; 
                                flag_fly[0]=0;

                        }
                            

                        if ( flag_st[0] == 1 )
                        {

                            ///////store the postion of joints to calculate the value of initiall error///////////
                            if (t_norminal[0] == 0)
                            {
                                Store_q_FRH=All_joint[0];
                                Store_q_FRT=All_joint[1];
                                Store_q_FRC=All_joint[2];
                            }
                            transition_rate = 1.0;

                            if (t_norminal_st[0] >= 0.70 && t_norminal_st[0] <= 0.8) St_sel_velocity = clamp(TorsoVelocity[0], x_velocity[0], x_velocity[x_velocity.size()-1]), trunk_Vel_fil[0] = St_sel_velocity;
                            
                            ////add the compensation torque for pitch velocity//since only one trigger has set up and sometime the other leg will be used as a trigger
                           // tao_thigh_pitch[0] = Kp_tao_thigh_pitch[0] * (trunk_YPR_fil[1]);
                            if (t_norminal_st[0] >= 0.0 && t_norminal_st[0] <= 0.8) {
                                tao_thigh_pitch[0] = tao_thigh_pitch[0];
                            }
                            else{
                                tao_thigh_pitch[0] = 0;
                            }
                            //////////////////////////////////////////////////

                            if (t_norminal_st[0] < 1.5){
                                pre_number = t_norminal_st[0]*1000*step_time_st;
                            }
                            else{
                                pre_number = 439;
                            }
                            int_pre_number = (int)(0.5+pre_number);
                       
                            /////////////parameters of trajecotry/////////////////////////////
                            coeff_st_FRH = coeff_st.row(0);

                            coeff_st_FRT = coeff_st.row(1);

                            coeff_st_FRC = coeff_st.row(2);

                            // coeff_st_FRT[0] = Store_q_FRT;
                            coeff_st_FRC[0] = Store_q_FRC;
                            // coeff_st_FRC[1] = Store_q_FRC - 0.2;

                            ///////////initial error////////
                            delta_q_FRH=fcn_bezier(coeff_st_FRH, 0)-Store_q_FRH;
                            delta_q_FRT=fcn_bezier(coeff_st_FRT, 0)-Store_q_FRT;
                            delta_q_FRC=fcn_bezier(coeff_st_FRC, 0)-Store_q_FRC;

                            /////use saturation function to reduce the error a step by step/////
                            q_joint[0] = fcn_bezier(coeff_st_FRH, t_norminal[0]);//-delta_q_FLH*(1-tanh(2*t_norminal[1]));
                            q_joint[1] = fcn_bezier(coeff_st_FRT, t_norminal[0]) - ConsOffset - hd_FR_LA_fil[0]*0.0;
                            q_joint[2] = fcn_bezier(coeff_st_FRC, t_norminal[0]);


                            ////generate the error for torques////tracking error///

                            // for(int i_motor = 0; i_motor < 3; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }
                
                            if (t_norminal_st[0] < 1.2)
                            {
                                dq_joint[0] = fcn_dbezier(coeff_st_FRH, t_norminal[0]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                                dq_joint[1] = fcn_dbezier(coeff_st_FRT, t_norminal[0]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                                dq_joint[2] = fcn_dbezier(coeff_st_FRC, t_norminal[0]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                            }
                            else
                            {
                                dq_joint[0] = 0;
                                dq_joint[1] = 0;
                                dq_joint[2] = 0;
                            }


                            // for(int e_j = 0; e_j < 3; e_j++){
                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]
                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 0; i_j < 3; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j]; 

                            if (touchdelay > 0 && t_norminal_st[0]< touchdelay)
                            {
                                t_norminal[0] = 0;
                            }
                            else
                            {
                                t_norminal[0] = min<float>(t_norminal_st[0], normaltime) - touchdelay;
                            } 


                            /////////keep the joint angles for the fly phase///////
                            for(int i_motor = 0; i_motor < 3; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// - 20;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 0; i_j < 3; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[0]))) ;
                            //     torque[i_j] = original_torque[i_j];

                            // }


                            if (t_norminal_st[0] < 0.5)
                            {
                                t_norminal_st[0] = t_norminal_st[0]+(t_now-t_pre[0])*step_ct_st;
                            }
                            else{
                                t_norminal_st[0] = t_norminal_st[0] + 1*(t_now-t_pre[0])*step_ct_st;
                            } 


                            /////////////////////////////////////////////////////////////////////////////////

                
                            for ( int i_tor = 0; i_tor < 12; i_tor++ ){
                                if (int_pre_number <=439){
                  
                                    per_st_motor_total_fil[i_tor] = total_stance_torque_fil[int_pre_number][i_tor];
                                    per_st_motor_torque_fil[i_tor] = pre_torque_stance_fil_zero[int_pre_number][i_tor];
                                    per_st_motor_fil_err_tor[i_tor] = pre_torque_stance_fil_err_tor[int_pre_number][i_tor];
                                }
                                else{

                                    per_st_motor_total_fil[i_tor] = total_stance_torque_fil[int_pre_number][i_tor];
                                    per_st_motor_torque_fil[i_tor] = pre_torque_stance_fil_zero[int_pre_number][i_tor];
                                    per_st_motor_fil_err_tor[i_tor] = pre_torque_stance_fil_err_tor[int_pre_number][i_tor];
                                }       
                            }

                            for ( int i_tor = 0; i_tor < 12; i_tor++ ){
                                if (int_pre_number <=199){
                                    per_st_Error[i_tor] = Pre_Error_stance[int_pre_number+30][i_tor];
                                    per_st_dError[i_tor] = Pre_dError_stance[int_pre_number+30][i_tor];

                                }
                                else{
                                    per_st_Error[i_tor] = Pre_Error_stance[238][i_tor];
                                    per_st_dError[i_tor] = Pre_dError_stance[238][i_tor];
                                }       
                            }
                            for ( int i_j = 0; i_j < 12; i_j++ )  Feedforward_Error_tor[i_j] = ILC_Switch[0] * (joint_Kp_fd[i_j] * per_st_Error[i_j] + joint_Kv_fd[i_j] * per_st_dError[i_j]) + 0.0 *ILC_Switch[1] * per_st_motor_fil_err_tor[i_j];

                            for ( int i_j = 0; i_j < 12; i_j++ ) {
                                if (t_norminal_st[0] <=0.5){
                                    Feedforward_Error_sat[i_j] = Feedforward_Error_tor[i_j]*tanh(90*(t_norminal_st[0])); // only for one contact
                                }
                                else{
                                    if (tanh(-5*(t_norminal_st[0]-1.3))>0){
                                        Feedforward_Error_sat[i_j] = Feedforward_Error_tor[i_j]*tanh(-5*(t_norminal_st[0]-1.2)); // only for one contact(in order to avoid the error of the torque)
                                    }
                                    else{
                                        Feedforward_Error_sat[i_j] = 0; // 1.3 means total stancpahase
                                    }
                                }
                            }


                            Store[0] = t_now-t_pre[0];                
                            std::cout<<"step    "<<Store[0]<< std::endl;
                            std::cout<<"flag_st    "<<"1"<<"     and     "<<"flag_fly    "<<"0"<< std::endl;
                            t_pre[0]=t_now;
                            std::cout << "Checking: t_norminal_st" << t_norminal_st[0]<<std::endl;
                            std::cout << "/////////////////////////" << std::endl; 

                            for ( int j_st = 0 ; j_st<12; j_st++){
                            torque_stance[int_pre_number][j_st] = RecvLowROS.motorState[j_st].tauEst;
                            Error_stance[int_pre_number][j_st] = Error_q[j_st];
                            dError_stance[int_pre_number][j_st] = Error_dq[j_st];
                            Feedforward_Error_tor_stance[int_pre_number][j_st] = Feedforward_Error_tor[j_st];
                            }   
                            // for (int i_st = 0; i_st < 439; i_st++){
                            //      std::cerr << "total_stance_torque_fil[0][0][50] = " << total_stance_torque_fil[i_st][2]  << std::endl;    
                            // }     
                            // std::cerr << "per_st_motor_total_fil[0][0][50] = " << per_st_motor_total_fil[2]  << std::endl;
                        }

                        else if ( flag_fly[0] == 1 )
                        {
                           
                            ///////store the postion of joints set the coefficients///////////
                            transition_rate = 0;
                            if (t_norminal[0] == 0)
                            {
                                Store_q_FRH=All_joint[0];
                                Store_q_FRT=All_joint[1];
                                Store_q_FRC=All_joint[2];
                            }
                            /////////////reset the additional torque in thigh in flight phase///////////////

                             tao_thigh_pitch[0] = 0.0f;
                             for ( int i_j = 0; i_j < 12; i_j++ )  Feedforward_Error_tor[i_j] =0.0f;
                             for ( int i_j = 0; i_j < 12; i_j++ )  Feedforward_Error_sat[i_j] = 0.0f;
                             for ( int i_j = 0; i_j < 12; i_j++ )  ILC_torque[i_j] =0.0f;


                            /////////////parameters of trajecotry in fly phase/////////////////////////////
                            coeff_fly_FRH = coeff_fly.row(0);

                            coeff_fly_FRT = coeff_fly.row(1);

                            coeff_fly_FRC = coeff_fly.row(2);


                            // coeff_fly_FRT[0] = Store_q_FRT;
                            // coeff_fly_FRC[0] = Store_q_FRC;

                            ///////////////////////////////////////////////////////////////////////
                            if (t_norminal_fly[0] >= 0.53 && t_norminal_fly[0] <= 0.57) fly_sel_velocity = clamp(trunk_Vel_strongfil[0], x_velocity[0], x_velocity[x_velocity.size()-1]);
                            cerr << "t_norminal_fly[0]" << t_norminal_fly[0] << endl;



                            for(int i_motor = 0; i_motor < 3; i_motor++){
                                // q_joint[i_motor] = fly_q[i_motor];
                                Kp[i_motor] = joint_Kp_f[i_motor];
                                Kd[i_motor] = joint_Kv_f[i_motor];
                            }

                            hd_FR_LA[0] = velocity_regulation(fly_sel_velocity, transition_rate_temp, desired_velocity_trunk, trunk_Vel_strongfil, motiontime);
                            first_order_filter(hd_FR_LA_fil, hd_FR_LA, para_hd_FR, hd_FR_arrSize);

                            q_joint[0] = fcn_bezier(coeff_fly_FRH, t_norminal[0]);
                            q_joint[1] = fcn_bezier(coeff_fly_FRT, t_norminal[0])- ConsOffset;
                            q_joint[2] = fcn_bezier(coeff_fly_FRC, t_norminal[0]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 0; i_motor < 3; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            // for(int e_j = 0; e_j < 3; e_j++) dq_joint[e_j] = 0;
                            //////this is used to control the velocity jump at the end of the stride                            
                            dq_joint[0] = 0;
                            if (t_norminal_fly[0] < 1)
                            { 
                                dq_joint[0] = fcn_dbezier(coeff_fly_FRC, t_norminal[0]) * step_ct_fly;                          
                                dq_joint[1] = fcn_dbezier(coeff_fly_FRT, t_norminal[0]) * step_ct_fly;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                                dq_joint[2] = fcn_dbezier(coeff_fly_FRC, t_norminal[0]) * step_ct_fly;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                            }
                            else{
                                dq_joint[0] = 0;
                                dq_joint[1] = 0;
                                dq_joint[2] = 0;
                            }

                            // for(int e_j = 0; e_j < 3; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                            // }

                            //////////////////////////////////////////////////////////////
                            /////////////  desired_velocity_trunk[0] =  0.0;//////////////
                            // for ( int i_j = 0; i_j < 3; i_j++ ) torque_q[i_j] = joint_Kp_f[i_j] * Error_q[i_j] + joint_Kv_f[i_j] * Error_dq[i_j];

                            // for ( int i_j = 0; i_j < 3; i_j++ ) {

                            //     torque[i_j] = torque_q[i_j] ;
                                
                            // }                            

                            t_norminal_fly[0]=t_norminal_fly[0]+(t_now-t_pre[0])*step_ct_fly;
                            t_norminal[0] = min<float>(t_norminal_fly[0], 1.01);



                            std::cout<<"t_norminal_fly    "<<t_norminal_fly[0]<< std::endl;
                            std::cout<<"flag_fly_FR       "<<flag_fly[0]<< std::endl;
                            std::cout<<"flag_st    "<<"0"<<"     and     "<<"flag_fly    "<<"1"<< std::endl; 

                            t_pre[0]=t_now;
                            
                            // std::cout << "/////////////////////////" << std::endl;  
                            // std::cout << "Checking: t_norminal_st" << t_norminal_st[0]<<std::endl;

                        }

                        else if ( flag_st[0] == 2 )
                        {
                            transition_rate = 1.0;
                            ///////store the postion of joints to calculate the value of initiall error///////////
                            if (t_norminal[0] == 0)
                            {
                                Store_q_FRH=All_joint[0];
                                Store_q_FRT=All_joint[1];
                                Store_q_FRC=All_joint[2];
                            }

                            
                            ////add the compensation torque for pitch velocity//since only one trigger has set up and sometime the other leg will be used as a trigger
                           // tao_thigh_pitch[0] = Kp_tao_thigh_pitch[0] * (trunk_YPR_fil[1]);
                            if (t_norminal_st[0] >= 0.0 && t_norminal_st[0] <= 0.8) {
                                tao_thigh_pitch[0] = tao_thigh_pitch[0];
                            }
                            else{
                                tao_thigh_pitch[0] = 0;
                            }
                            //////////////////////////////////////////////////

                            if (t_norminal_st[0] < 1.5){
                                pre_number = t_norminal_st[0]*1000*step_time_stouch;
                            }
                            else{
                                pre_number = 519;
                            }
                            int_pre_number = (int)(0.5+pre_number);
                       
                            /////////////parameters of trajecotry/////////////////////////////
                            coeff_stouch_FRH = coeff_stouch.row(0);

                            coeff_stouch_FRT = coeff_stouch.row(1);

                            coeff_stouch_FRC = coeff_stouch.row(2);

                            coeff_stouch_FRT[0] = Store_q_FRT;
                            coeff_stouch_FRC[0] = Store_q_FRC;
                            // coeff_stouch_FRC[1] = Store_q_FRC - 0.2;

                            ///////////initial error////////
                            delta_q_FRH=fcn_bezier(coeff_stouch_FRH, 0)-Store_q_FRH;
                            delta_q_FRT=fcn_bezier(coeff_stouch_FRT, 0)-Store_q_FRT;
                            delta_q_FRC=fcn_bezier(coeff_stouch_FRC, 0)-Store_q_FRC;

                            /////use saturation function to reduce the error a step by step/////
                            q_joint[0] = fcn_bezier(coeff_stouch_FRH, t_norminal[0]);
                            q_joint[1] = fcn_bezier(coeff_stouch_FRT, t_norminal[0]);
                            q_joint[2] = fcn_bezier(coeff_stouch_FRC, t_norminal[0]);


                            ////generate the error for torques////tracking error///

                            // for(int i_motor = 0; i_motor < 3; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            if (t_norminal_st[0] < 1)
                            {
                                dq_joint[0] = fcn_dbezier(coeff_stouch_FRH, t_norminal[0]) * step_ct_stouch;
                                dq_joint[1] = fcn_dbezier(coeff_stouch_FRT, t_norminal[0]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                                dq_joint[2] = fcn_dbezier(coeff_stouch_FRC, t_norminal[0]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[0])));
                            }
                            else
                            {
                                dq_joint[0] = 0;
                                dq_joint[1] = 0;
                                dq_joint[2] = 0;
                            }


                            // for(int e_j = 0; e_j < 3; e_j++){
                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]
                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 0; i_j < 3; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j]; 

                            if (touchdelay > 0 && t_norminal_st[0]< touchdelay)
                            {
                                t_norminal[0] = 0;
                            }
                            else
                            {
                                t_norminal[0] = min<float>(t_norminal_st[0], normaltime) - touchdelay;
                            } 


                            /////////keep the joint angles for the fly phase///////
                            for(int i_motor = 0; i_motor < 3; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// - 20;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 0; i_j < 3; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[0]))) ;
                            //     torque[i_j] = original_torque[i_j];

                            // }


                            if (t_norminal_st[0] < 0.5)
                            {
                                t_norminal_st[0] = t_norminal_st[0]+(t_now-t_pre[0])*step_ct_stouch;
                            }
                            else{
                                t_norminal_st[0] = t_norminal_st[0] + 1*(t_now-t_pre[0])*step_ct_stouch;
                            } 


                            /////////////////////////////////////////////////////////////////////////////////

                
                            for ( int i_tor = 0; i_tor < 12; i_tor++ ){
                                if (int_pre_number <=519){
                  
                                    per_stouch_motor_total_fil[i_tor] = total_stouch_torque_fil[int_pre_number][i_tor];
                                    per_stouch_motor_torque_fil[i_tor] = pre_torque_stance_fil_zero[int_pre_number][i_tor];
                                    per_stouch_motor_fil_err_tor[i_tor] = pre_torque_stance_fil_err_tor[int_pre_number][i_tor];
                                }
                                else{

                                    per_stouch_motor_total_fil[i_tor] = total_stouch_torque_fil[519][i_tor];
                                    per_stouch_motor_torque_fil[i_tor] = pre_torque_stance_fil_zero[519][i_tor];
                                    per_stouch_motor_fil_err_tor[i_tor] = pre_torque_stance_fil_err_tor[519][i_tor];
                                }       
                            }

                            for ( int i_tor = 0; i_tor < 12; i_tor++ ){
                                if (int_pre_number <=489){
                                    per_stouch_Error[i_tor] = Pre_Error_stance[int_pre_number+30][i_tor];
                                    per_stouch_dError[i_tor] = Pre_dError_stance[int_pre_number+30][i_tor];

                                }
                                else{
                                    per_stouch_Error[i_tor] = Pre_Error_stance[519][i_tor];
                                    per_stouch_dError[i_tor] = Pre_dError_stance[519][i_tor];
                                }       
                            }
                            for ( int i_j = 0; i_j < 12; i_j++ )  Feedforward_Error_tor[i_j] = ILC_Switch[0] * (joint_Kp_fd[i_j] * per_stouch_Error[i_j] + joint_Kv_fd[i_j] * per_stouch_dError[i_j]) + 0.0 *ILC_Switch[1] * per_stouch_motor_fil_err_tor[i_j];

                            for ( int i_j = 0; i_j < 12; i_j++ ) {
                                if (t_norminal_st[0] <=0.5){
                                    Feedforward_Error_sat[i_j] = Feedforward_Error_tor[i_j]*tanh(90*(t_norminal_st[0])); // only for one contact
                                }
                                else{
                                    if (tanh(-5*(t_norminal_st[0]-1.3))>0){
                                        Feedforward_Error_sat[i_j] = Feedforward_Error_tor[i_j]*tanh(-5*(t_norminal_st[0]-1.2)); // only for one contact(in order to avoid the error of the torque)
                                    }
                                    else{
                                        Feedforward_Error_sat[i_j] = 0; // 1.3 means total stancpahase
                                    }
                                }
                            }


                            Store[0] = t_now-t_pre[0]; 
                            t_pre[0]=t_now;  




                        for ( int j_st = 0 ; j_st<12; j_st++){
                            torque_stouch[int_pre_number][j_st] = 0;
                            Error_stouch[int_pre_number][j_st] = 0;
                            dError_stouch[int_pre_number][j_st] = 0;
                            Feedforward_Error_tor_stouch[int_pre_number][j_st] = 0;
                        }

                        }
                         ////////////////////////////  FL  //////////////////
                        if ( t_now < 0.005 )
                        {
                            t_norminal_st[1] = 0;
                            t_norminal_fly[1] = 0;
                            t_norminal[1] = 0;
                            flag_st[1]=1;  
                            flag_fly[1]=0;
                        }
                        else if ( (t_norminal_st[1] > st_switch &&  trigger_forceF <= 0 && t_now < 0.6 ) )
                        {

                            t_norminal_fly[1] = 0;
                            t_norminal_st[1] = 0;
                            t_norminal[1] = 0;
                            flag_st[1]=0;  
                            flag_fly[1]=1;
                        }
                        else if ( (t_norminal_fly[1] > fly_switch &&  trigger_forceR > 20) )
                        {
                            t_norminal_st[1] = 0;
                            t_norminal_fly[1] = 0;
                            t_norminal[1] = 0;
                            flag_st[1]=2; 
                            flag_fly[1]=0;
                        }

                        if ( flag_st[1] == 1 )
                        {
                
                            if (t_norminal[1] == 0){
                                Store_q_FLH=All_joint[3];
                                Store_q_FLT=All_joint[4];
                                Store_q_FLC=All_joint[5]; 
                            }

                            /////////////////////////////////
                            coeff_st_FLH = coeff_st.row(3);

                            coeff_st_FLT = coeff_st.row(4);

                            coeff_st_FLC = coeff_st.row(5);                           

                            // coeff_st_FLT[0] = Store_q_FLT;
                            coeff_st_FLC[0] = Store_q_FLC;
                            // coeff_st_FLC[1] = Store_q_FLC - 0.2;               

                            delta_q_FLH=fcn_bezier(coeff_st_FLH, 0)-Store_q_FLH;
                            delta_q_FLT=fcn_bezier(coeff_st_FLT, 0)-Store_q_FLT;
                            delta_q_FLC=fcn_bezier(coeff_st_FLC, 0)-Store_q_FLC;

                            q_joint[3]=fcn_bezier(coeff_st_FLH, t_norminal[1]);
                            q_joint[4]=fcn_bezier(coeff_st_FLT, t_norminal[1]) - ConsOffset - hd_FR_LA_fil[0]*0.0;
                            q_joint[5]=fcn_bezier(coeff_st_FLC, t_norminal[1]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 3; i_motor < 6; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                           
                            if (t_norminal_st[1] < 1.2)
                            {    
                                dq_joint[3] = fcn_dbezier(coeff_st_FLH, t_norminal[1]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));                     
                                dq_joint[4] = fcn_dbezier(coeff_st_FLT, t_norminal[1]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));
                                dq_joint[5] = fcn_dbezier(coeff_st_FLC, t_norminal[1]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));
                            }
                            else{
                                dq_joint[3] = 0;
                                dq_joint[4] = 0;
                                dq_joint[5] = 0;
                            }

                            //tao_thigh_pitch[1] = Kp_tao_thigh_pitch[1] * (trunk_YPR_fil[1]);
                            if (t_norminal_st[1] >= 0.0 && t_norminal_st[1] <= 0.8) {
                                tao_thigh_pitch[1] = tao_thigh_pitch[1];
                            }
                            else{
                                tao_thigh_pitch[1] = 0;
                            }
                            // for(int e_j = 3; e_j < 6; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 3; i_j < 6; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];

                            if (touchdelay > 0 && t_norminal_st[1]< touchdelay)
                            {
                                t_norminal[1] = 0;
                            }
                            else
                            {
                                t_norminal[1] = min<float>(t_norminal_st[1], normaltime) - touchdelay;
                            }
                            
                            /////////keep the joint angles for the fly phase and set the gains in stand phase///////
                            for(int i_motor = 3; i_motor < 6; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// - 20;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 3; i_j < 6; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[1]))) ;
                            //     torque[i_j] = original_torque[i_j];

                            // }                         


                            if (t_norminal_st[1] < 0.5)
                            {
                                t_norminal_st[1] = t_norminal_st[1]+(t_now-t_pre[1])*step_ct_st;
                            }
                            else{
                                t_norminal_st[1] = t_norminal_st[1] + 1*(t_now-t_pre[1])*step_ct_st;
                            }
                            Store[1] = t_now-t_pre[1];                
                            t_pre[1]=t_now;

                
                        }

                        else if ( flag_fly[1] == 1 )
                        {
                            if (t_norminal[1] == 0)
                            {
                                Store_q_FLH=All_joint[3];
                                Store_q_FLT=All_joint[4];
                                Store_q_FLC=All_joint[5];
                            }
            
                            /////////////parameters of trajecotry in fly phase/////////////////////////////
                            coeff_fly_FLH = coeff_fly.row(0);

                            coeff_fly_FLT = coeff_fly.row(1);

                            coeff_fly_FLC = coeff_fly.row(2); 

                            // coeff_fly_FLT[0] = Store_q_FLT;
                            // coeff_fly_FLC[0] = Store_q_FLC;
                            /////////////////////////////////////////////////////////
                            tao_thigh_pitch[1] = 0.0f;                        

                            for(int i_motor = 3; i_motor < 6; i_motor++){
                                // q_joint[i_motor] = fly_q[i_motor];
                                Kp[i_motor] = joint_Kp_f[i_motor];
                                Kd[i_motor] = joint_Kv_f[i_motor];
                            }

                            q_joint[3] = fcn_bezier(coeff_fly_FLH, t_norminal[1]);
                            q_joint[4] = fcn_bezier(coeff_fly_FLT, t_norminal[1]) - ConsOffset;
                            q_joint[5] = fcn_bezier(coeff_fly_FLC, t_norminal[1]);                               

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 3; i_motor < 6; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            // for(int e_j = 3; e_j < 6; e_j++) dq_joint[e_j] = 0;
                            if (t_norminal_fly[1] < 1)
                            {    
                                dq_joint[3] = fcn_dbezier(coeff_fly_FLH, t_norminal[1]) * step_ct_fly;                      
                                dq_joint[4] =  fcn_dbezier(coeff_fly_FLT, t_norminal[1]) * step_ct_fly;
                                dq_joint[5] =  fcn_dbezier(coeff_fly_FLC, t_norminal[1]) * step_ct_fly;
                            }
                            else{
                                dq_joint[3] = 0;
                                dq_joint[4] = 0;
                                dq_joint[5] = 0;
                            }


                            // for(int e_j = 3; e_j < 6; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                            // }

                            // for ( int i_j = 3; i_j < 6; i_j++ ) torque_q[i_j] = joint_Kp_f[i_j] * Error_q[i_j] + joint_Kv_f[i_j] * Error_dq[i_j];

                            // for ( int i_j = 3; i_j < 6; i_j++ ) {

                            //     torque[i_j] = torque_q[i_j] ;
                                
                            // }                            

                            t_norminal_fly[1]=t_norminal_fly[1]+(t_now-t_pre[1])*step_ct_fly;
                            t_norminal[1] = min<float>(t_norminal_fly[1], 1.01);


                            t_pre[1]=t_now;
                        }

                        else if ( flag_st[1] == 2 )
                        {
                
                            if (t_norminal[1] == 0){
                                Store_q_FLH=All_joint[3];
                                Store_q_FLT=All_joint[4];
                                Store_q_FLC=All_joint[5]; 
                            }

                            /////////////////////////////////
                            coeff_st_FLH = coeff_stouch.row(3);

                            coeff_st_FLT = coeff_stouch.row(4);

                            coeff_st_FLC = coeff_stouch.row(5);                           

                            coeff_st_FLT[0] = Store_q_FLT;
                            coeff_st_FLC[0] = Store_q_FLC;             

                            delta_q_FLH=fcn_bezier(coeff_st_FLH, 0)-Store_q_FLH;
                            delta_q_FLT=fcn_bezier(coeff_st_FLT, 0)-Store_q_FLT;
                            delta_q_FLC=fcn_bezier(coeff_st_FLC, 0)-Store_q_FLC;

                            q_joint[3]=fcn_bezier(coeff_st_FLH, t_norminal[1]);
                            q_joint[4]=fcn_bezier(coeff_st_FLT, t_norminal[1]);
                            q_joint[5]=fcn_bezier(coeff_st_FLC, t_norminal[1]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 3; i_motor < 6; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                           
                            if (t_norminal_st[1] < 1)
                            {    
                                dq_joint[3] = fcn_dbezier(coeff_st_FLH, t_norminal[1]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));                     
                                dq_joint[4] = fcn_dbezier(coeff_st_FLT, t_norminal[1]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));
                                dq_joint[5] = fcn_dbezier(coeff_st_FLC, t_norminal[1]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[1])));
                            }
                            else{
                                dq_joint[3] = 0;
                                dq_joint[4] = 0;
                                dq_joint[5] = 0;
                            }

                            //tao_thigh_pitch[1] = Kp_tao_thigh_pitch[1] * (trunk_YPR_fil[1]);
                            if (t_norminal_st[1] >= 0.0 && t_norminal_st[1] <= 0.8) {
                                tao_thigh_pitch[1] = tao_thigh_pitch[1];
                            }
                            else{
                                tao_thigh_pitch[1] = 0;
                            }
                            // for(int e_j = 3; e_j < 6; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 3; i_j < 6; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];

                            if (touchdelay > 0 && t_norminal_st[1]< touchdelay)
                            {
                                t_norminal[1] = 0;
                            }
                            else
                            {
                                t_norminal[1] = min<float>(t_norminal_st[1], normaltime) - touchdelay;
                            }
                            
                            /////////keep the joint angles for the fly phase and set the gains in stand phase///////
                            for(int i_motor = 3; i_motor < 6; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// - 20;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 3; i_j < 6; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[1]))) ;
                            //     torque[i_j] = original_torque[i_j];

                            // }                         


                            if (t_norminal_st[1] < 0.5)
                            {
                                t_norminal_st[1] = t_norminal_st[1]+(t_now-t_pre[1])*step_ct_stouch;
                            }
                            else{
                                t_norminal_st[1] = t_norminal_st[1] + 1*(t_now-t_pre[1])*step_ct_stouch;
                            }
                            Store[1] = t_now-t_pre[1];                
                            t_pre[1]=t_now;

                
                        }
                        ////////////////////////RR////////////////////
                        if ( t_now < 0.005 )
                        {
                            t_norminal_st[2] = 0;
                            t_norminal_fly[2] = 0;
                            t_norminal[2] = 0;
                            flag_st[2]=1;  
                            flag_fly[2]=0;
                        }
                        else if ( (t_norminal_st[2] > st_switch &&  trigger_forceF <= 0 && t_now < 0.6) )
                        {
                            t_norminal_fly[2] = 0;
                            t_norminal_st[2] = 0;
                            t_norminal[2] = 0;
                            flag_st[2]=0;  
                            flag_fly[2]=1;
                        }
                        else if ( (t_norminal_fly[2] > fly_switch &&  trigger_forceR > 20) )
                        {
                            t_norminal_st[2] = 0;
                            t_norminal_fly[2] = 0;
                            t_norminal[2] = 0;
                            flag_st[2]=2; 
                            flag_fly[2]=0;
                        }

                        if ( flag_st[2] == 1 )
                        {
                            if (t_norminal[2] == 0){
                                Store_q_RRH=All_joint[6];
                                Store_q_RRT=All_joint[7];
                                Store_q_RRC=All_joint[8]; 
                            }
                        /////////////////////////////////
                            coeff_st_RRH = coeff_st.row(6);

                            coeff_st_RRT = coeff_st.row(7);

                            coeff_st_RRC = coeff_st.row(8);             

                            // coeff_st_RRT[0] = Store_q_RRT;
                            // coeff_st_RRT[1] = Store_q_RRT + 0.2;

                            coeff_st_RRC[0] = Store_q_RRC;
                            // coeff_st_RRC[1] = Store_q_RRC - 0.2;


                            delta_q_RRH=fcn_bezier(coeff_st_RRH, 0)-Store_q_RRH;
                            delta_q_RRT=fcn_bezier(coeff_st_RRT, 0)-Store_q_RRT;
                            delta_q_RRC=fcn_bezier(coeff_st_RRC, 0)-Store_q_RRC;

                            q_joint[6]= fcn_bezier(coeff_st_RRH, t_norminal[2]);
                            q_joint[7]=fcn_bezier(coeff_st_RRT, t_norminal[2]) - ConsOffset - hd_FR_LA_fil[0]*0.0;
                            q_joint[8]=fcn_bezier(coeff_st_RRC, t_norminal[2]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 6; i_motor < 9; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            if (t_norminal_st[2] < 1.2)
                            {   
                                dq_joint[6] = fcn_dbezier(coeff_st_RRH, t_norminal[2]) * step_ct_st;  
                                dq_joint[7] = fcn_dbezier(coeff_st_RRT, t_norminal[2]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[2])));
                                dq_joint[8] = fcn_dbezier(coeff_st_RRC, t_norminal[2]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[2])));
                            }
                            else{
                                dq_joint[6] = 0;
                                dq_joint[7] = 0;
                                dq_joint[8] = 0;
                            }
                            // for(int e_j = 6; e_j < 9; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 6; i_j < 9; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];      
                            ////////work after touch on the ground
                            if (t_norminal_st[2] >= 0.0 && t_norminal_st[2] <= 0.8) {
                                tao_thigh_pitch[2] = tao_thigh_pitch[2];
                            }
                            else{
                                tao_thigh_pitch[2] = 0;
                            }                    

                            if (touchdelay > 0 && t_norminal_st[2]< touchdelay)
                            {
                                t_norminal[2] = 0;
                            }
                            else
                            {
                                t_norminal[2] = min<float>(t_norminal_st[2], normaltime) - touchdelay;
                            }

                            /////////keep the joint angles for the fly phase and set the gains in stance phase///////
                            for(int i_motor = 6; i_motor < 9; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// + 30;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 6; i_j < 9; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[2])));
                            //     torque[i_j] = original_torque[i_j];

                            // }                        

                            if (t_norminal_st[2] < 0.5)
                            {
                                t_norminal_st[2] = t_norminal_st[2]+(t_now-t_pre[2])*step_ct_st;
                            }
                            else{
                                t_norminal_st[2] = t_norminal_st[2] + 1*(t_now-t_pre[2])*step_ct_st;
                            }                

                            Store[2] = t_now-t_pre[2];                
                            cout<<"step    "<<Store[2]<<endl;
                            t_pre[2]=t_now;      
                        }

                        else if ( flag_fly[2] == 1 )
                        { 
                            if (t_norminal[2] == 0)
                            {
                                Store_q_RRH=All_joint[6];
                                Store_q_RRT=All_joint[7];
                                Store_q_RRC=All_joint[8];
                            }
                            /////////////parameters of trajecotry in fly phase/////////////////////////////
                            coeff_fly_RRH = coeff_fly.row(6);

                            coeff_fly_RRT = coeff_fly.row(7);

                            coeff_fly_RRC = coeff_fly.row(8);

                            // coeff_fly_RRT[0] = Store_q_RRT;
                            // coeff_fly_RRC[0] = Store_q_RRC;
                            /////////////////////////////////////////////////////////
                            tao_thigh_pitch[2] = 0.0f;



                            for(int i_motor = 6; i_motor < 9; i_motor++){
                                // q_joint[i_motor] = fly_q[i_motor];
                                Kp[i_motor] = joint_Kp_f[i_motor];
                                Kd[i_motor] = joint_Kv_f[i_motor];
                            }

                            q_joint[6] = fcn_bezier(coeff_fly_RRH, t_norminal[2]);
                            q_joint[7] = fcn_bezier(coeff_fly_RRT, t_norminal[2]) - ConsOffset;
                            q_joint[8] = fcn_bezier(coeff_fly_RRC, t_norminal[2]);
                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 6; i_motor < 9; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            // for(int e_j = 6; e_j < 9; e_j++) dq_joint[e_j] = 0;
                            
                            if (t_norminal_fly[2] < 1)
                            {     
                                dq_joint[6] = fcn_dbezier(coeff_fly_RRH, t_norminal[2]) * step_ct_fly;
                                dq_joint[7] = fcn_dbezier(coeff_fly_RRT, t_norminal[2]) * step_ct_fly;
                                dq_joint[8] = fcn_dbezier(coeff_fly_RRC, t_norminal[2]) * step_ct_fly;
                            }
                            else{
                                dq_joint[6] = 0;
                                dq_joint[7] = 0;
                                dq_joint[8] = 0;
                            }
                            // for(int e_j = 6; e_j < 9; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                            // }

                            // for ( int i_j = 6; i_j < 9; i_j++ ) torque_q[i_j] = joint_Kp_f[i_j] * Error_q[i_j] + joint_Kv_f[i_j] * Error_dq[i_j];

                            // for ( int i_j = 6; i_j < 9; i_j++ ) {

                            //     torque[i_j] = torque_q[i_j] ;
                                
                            // }                                                    

                            t_norminal_fly[2] = t_norminal_fly[2]+(t_now-t_pre[2])*step_ct_fly;
                            t_norminal[2] = min<float>(t_norminal_fly[2], 1.01);

                            t_pre[2]=t_now;
                        }
                        else if ( flag_st[2] == 2 )
                        {
                            if (t_norminal[2] == 0){
                                Store_q_RRH=All_joint[6];
                                Store_q_RRT=All_joint[7];
                                Store_q_RRC=All_joint[8]; 
                            }
                        /////////////////////////////////
                            coeff_st_RRH = coeff_stouch.row(6);

                            coeff_st_RRT = coeff_stouch.row(7);

                            coeff_st_RRC = coeff_stouch.row(8);             

                            coeff_st_RRT[0] = Store_q_RRT;
                            // coeff_st_RRT[1] = Store_q_RRT + 0.2;

                            coeff_st_RRC[0] = Store_q_RRC;
                            // coeff_st_RRC[1] = Store_q_RRC - 0.2;


                            delta_q_RRH=fcn_bezier(coeff_st_RRH, 0)-Store_q_RRH;
                            delta_q_RRT=fcn_bezier(coeff_st_RRT, 0)-Store_q_RRT;
                            delta_q_RRC=fcn_bezier(coeff_st_RRC, 0)-Store_q_RRC;

                            q_joint[6]= fcn_bezier(coeff_st_RRH, t_norminal[2]);
                            q_joint[7]=fcn_bezier(coeff_st_RRT, t_norminal[2]);
                            q_joint[8]=fcn_bezier(coeff_st_RRC, t_norminal[2]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 6; i_motor < 9; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            if (t_norminal_st[2] < 1)
                            {   
                                dq_joint[6] = fcn_dbezier(coeff_st_RRH, t_norminal[2]) * step_ct_stouch;  
                                dq_joint[7] = fcn_dbezier(coeff_st_RRT, t_norminal[2]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[2])));
                                dq_joint[8] = fcn_dbezier(coeff_st_RRC, t_norminal[2]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[2])));
                            }
                            else{
                                dq_joint[6] = 0;
                                dq_joint[7] = 0;
                                dq_joint[8] = 0;
                            }
                            // for(int e_j = 6; e_j < 9; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 6; i_j < 9; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];      
                            ////////work after touch on the ground
                            if (t_norminal_st[2] >= 0.0 && t_norminal_st[2] <= 0.8) {
                                tao_thigh_pitch[2] = tao_thigh_pitch[2];
                            }
                            else{
                                tao_thigh_pitch[2] = 0;
                            }                    

                            if (touchdelay > 0 && t_norminal_st[2]< touchdelay)
                            {
                                t_norminal[2] = 0;
                            }
                            else
                            {
                                t_norminal[2] = min<float>(t_norminal_st[2], normaltime) - touchdelay;
                            }

                            /////////keep the joint angles for the fly phase and set the gains in stance phase///////
                            for(int i_motor = 6; i_motor < 9; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// + 30;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 6; i_j < 9; i_j++ ) {

                            //     original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[2])));
                            //     torque[i_j] = original_torque[i_j];

                            // }                        

                            if (t_norminal_st[2] < 0.5)
                            {
                                t_norminal_st[2] = t_norminal_st[2]+(t_now-t_pre[2])*step_ct_stouch;
                            }
                            else{
                                t_norminal_st[2] = t_norminal_st[2] + 1*(t_now-t_pre[2])*step_ct_stouch;
                            }                

                            Store[2] = t_now-t_pre[2];                
                            cout<<"step    "<<Store[2]<<endl;
                            t_pre[2]=t_now;      
                        }
 
                        ////////////////////////RL//////////////////////////////////////
                        if (t_now < 0.005 )
                        {
                            t_norminal_st[3] = 0;
                            t_norminal_fly[3] = 0;
                            t_norminal[3] = 0;
                            flag_st[3]=1;  
                            flag_fly[3]=0;
                        }
                        else if ( (t_norminal_st[3] > st_switch &&  trigger_forceF <= 0 && t_now < 0.6 ) )
                        {
                            t_norminal_fly[3] = 0;
                            t_norminal_st[3] = 0;
                            t_norminal[3] = 0;
                            flag_st[3]=0;  
                            flag_fly[3]=1;
                        }
                        else if ( (t_norminal_fly[3] > fly_switch &&  trigger_forceR > 20) )
                        {
                            t_norminal_st[3] = 0;
                            t_norminal_fly[3] = 0;
                            t_norminal[3] = 0;
                            flag_st[3]=2; 
                            flag_fly[3]=0;
                        }

                        if ( flag_st[3] == 1 )
                        {
                            if (t_norminal[3] == 0){
                                Store_q_RLH=All_joint[9];
                                Store_q_RLT=All_joint[10];
                                Store_q_RLC=All_joint[11]; 
                            }
                        /////////////////////////////////
                            coeff_st_RLH = coeff_st.row(6);

                            coeff_st_RLT = coeff_st.row(7);

                            coeff_st_RLC = coeff_st.row(8);                     
                            // coeff_st_RLT[0] = Store_q_RLT;
                            // coeff_st_RLT[1] = Store_q_RLT + 0.2;

                            coeff_st_RLC[0] = Store_q_RLC;
                            // coeff_st_RLC[1] = Store_q_RLC - 0.2;

                            delta_q_RLH=fcn_bezier(coeff_st_RLH, 0)-Store_q_RLH;
                            delta_q_RLT=fcn_bezier(coeff_st_RLT, 0)-Store_q_RLT;
                            delta_q_RLC=fcn_bezier(coeff_st_RLC, 0)-Store_q_RLC;

                            q_joint[9]=fcn_bezier(coeff_st_RLH, t_norminal[3]);
                            q_joint[10]=fcn_bezier(coeff_st_RLT, t_norminal[3]) - ConsOffset - hd_FR_LA_fil[0]*0.0;
                            q_joint[11]=fcn_bezier(coeff_st_RLC, t_norminal[3]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 9; i_motor < 12; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }
 
                            if (t_norminal_st[3] < 1.2)
                            {
                                dq_joint[9] = fcn_dbezier(coeff_st_RLH, t_norminal[3]) * step_ct_st;
                                dq_joint[10] = fcn_dbezier(coeff_st_RLT, t_norminal[3]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[3])));
                                dq_joint[11] = fcn_dbezier(coeff_st_RLC, t_norminal[3]) * step_ct_st;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[3])));
                            }
                            else{
                                dq_joint[9] = 0;
                                dq_joint[10] = 0;
                                dq_joint[11] = 0;
                            }
                            // for(int e_j = 9; e_j < 12; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 9; i_j < 12; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];
                            if (t_norminal_st[3] >= 0.0 && t_norminal_st[3] <= 0.8) {
                                tao_thigh_pitch[3] = tao_thigh_pitch[3];
                            }
                            else{
                                tao_thigh_pitch[3] = 0;
                            }    

                            if (touchdelay > 0 && t_norminal_st[3]< touchdelay)
                            {
                                t_norminal[3] = 0;
                            }
                            else
                            {
                                t_norminal[3] = min<float>(t_norminal_st[3], normaltime) - touchdelay;
                            } 

                            /////////keep the joint angles for the fly phase///////
                            for(int i_motor = 9; i_motor < 12; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// + 30;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 9; i_j < 12; i_j++ ) {

                            //  original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[3])));
                            //     torque[i_j] = original_torque[i_j];

                            // }
                            
                            if (t_norminal_st[3] < 0.5)
                            {
                                t_norminal_st[3] = t_norminal_st[3]+(t_now-t_pre[3])*step_ct_st;
                            }
                            else{
                                t_norminal_st[3] = t_norminal_st[3] + 1*(t_now-t_pre[3])*step_ct_st;
                            }
                            
                            Store[3] = t_now-t_pre[3];                 
                            t_pre[3]=t_now; 
                        }

                        else if ( flag_fly[3] == 1 )
                        {
                            if (t_norminal[3] == 0)
                            {
                                Store_q_RLH=All_joint[9];
                                Store_q_RLT=All_joint[10];
                                Store_q_RLC=All_joint[11];
                            }
                            /////////////parameters of trajecotry in fly phase/////////////////////////////
                            coeff_fly_RLH = coeff_fly.row(6);

                            coeff_fly_RLT = coeff_fly.row(7);

                            coeff_fly_RLC = coeff_fly.row(8);

                            // coeff_fly_RLT[0] = Store_q_RLT;
                            // coeff_fly_RLC[0] = Store_q_RLC;
                            ///////////////////////////////////////////////////////////////////////////
                            tao_thigh_pitch[3] = 0.0f;

                            q_joint[9] = fcn_bezier(coeff_fly_RLH, t_norminal[3]);
                            q_joint[10] = fcn_bezier(coeff_fly_RLT, t_norminal[3]) - ConsOffset;
                            q_joint[11] = fcn_bezier(coeff_fly_RLC, t_norminal[3]);

                            for(int i_motor = 9; i_motor < 12; i_motor++){
                                // q_joint[i_motor] = fly_q[i_motor];
                                Kp[i_motor] = joint_Kp_f[i_motor];
                                Kd[i_motor] = joint_Kv_f[i_motor];
                            }

                            q_joint[9] = 0;
                            q_joint[10] = q_joint[10] - hd_FR_LA_fil[0]*0.0;
                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 9; i_motor < 12; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }

                            // for(int e_j = 9; e_j < 12; e_j++) dq_joint[e_j] = 0;
                            dq_joint[9] = 0;
                            if (t_norminal_fly[3] < 1)
                            {
                                dq_joint[9] = fcn_dbezier(coeff_fly_RLH, t_norminal[3]) * step_ct_fly;
                                dq_joint[10] = fcn_dbezier(coeff_fly_RLT, t_norminal[3]) * step_ct_fly;
                                dq_joint[11] = fcn_dbezier(coeff_fly_RLC, t_norminal[3]) * step_ct_fly;
                            }
                            else{
                                dq_joint[9] = 0;
                                dq_joint[10] = 0;
                                dq_joint[11] = 0;
                            }
                            // for(int e_j = 9; e_j < 12; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];
                            // }

                            // for ( int i_j = 9; i_j < 12; i_j++ ) torque_q[i_j] = joint_Kp_f[i_j] * Error_q[i_j] + joint_Kv_f[i_j] * Error_dq[i_j];

                            // for ( int i_j = 9; i_j < 12; i_j++ ) {

                            //     torque[i_j] = torque_q[i_j] ;
                                
                            // }

                            t_norminal_fly[3]=t_norminal_fly[3]+(t_now-t_pre[3])*step_ct_fly;
                            t_norminal[3] = min<float>(t_norminal_fly[3], 1.01);

                            t_pre[3]=t_now;
                        }

                        else if ( flag_st[3] == 2 )
                        {
                            if (t_norminal[3] == 0){
                                Store_q_RLH=All_joint[9];
                                Store_q_RLT=All_joint[10];
                                Store_q_RLC=All_joint[11]; 
                            }
                        /////////////////////////////////
                            coeff_st_RLH = coeff_stouch.row(9);

                            coeff_st_RLT = coeff_stouch.row(10);

                            coeff_st_RLC = coeff_stouch.row(11);                     
                            coeff_st_RLT[0] = Store_q_RLT;
                            // coeff_st_RLT[1] = Store_q_RLT + 0.2;

                            coeff_st_RLC[0] = Store_q_RLC;
                            // coeff_st_RLC[1] = Store_q_RLC - 0.2;

                            delta_q_RLH=fcn_bezier(coeff_st_RLH, 0)-Store_q_RLH;
                            delta_q_RLT=fcn_bezier(coeff_st_RLT, 0)-Store_q_RLT;
                            delta_q_RLC=fcn_bezier(coeff_st_RLC, 0)-Store_q_RLC;

                            q_joint[9]=fcn_bezier(coeff_st_RLH, t_norminal[3]);
                            q_joint[10]=fcn_bezier(coeff_st_RLT, t_norminal[3]);
                            q_joint[11]=fcn_bezier(coeff_st_RLC, t_norminal[3]);

                            ////generate the error for torques////tracking error///
                            // for(int i_motor = 9; i_motor < 12; i_motor++){
                            //     Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                            // }
 
                            if (t_norminal_st[3] < 1)
                            {
                                dq_joint[9] = fcn_dbezier(coeff_st_RLH, t_norminal[3]) * step_ct_stouch;
                                dq_joint[10] = fcn_dbezier(coeff_st_RLT, t_norminal[3]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[3])));
                                dq_joint[11] = fcn_dbezier(coeff_st_RLC, t_norminal[3]) * step_ct_stouch;// *  0.5 * (1 - tanh(50*(0.05 - 0.5 * t_norminal[3])));
                            }
                            else{
                                dq_joint[9] = 0;
                                dq_joint[10] = 0;
                                dq_joint[11] = 0;
                            }
                            // for(int e_j = 9; e_j < 12; e_j++){

                            //     Error_dq[e_j] = dq_joint[e_j] - All_joint_velfil[e_j];//dq_joint[e_j]

                            // }

                            ////calculate the original torqes

                            // for ( int i_j = 9; i_j < 12; i_j++ ) torque_q[i_j] = joint_Kp[i_j] * Error_q[i_j] + joint_Kv[i_j] * Error_dq[i_j];
                            if (t_norminal_st[3] >= 0.0 && t_norminal_st[3] <= 0.8) {
                                tao_thigh_pitch[3] = tao_thigh_pitch[3];
                            }
                            else{
                                tao_thigh_pitch[3] = 0;
                            }    

                            if (touchdelay > 0 && t_norminal_st[3]< touchdelay)
                            {
                                t_norminal[3] = 0;
                            }
                            else
                            {
                                t_norminal[3] = min<float>(t_norminal_st[3], normaltime) - touchdelay;
                            } 

                            /////////keep the joint angles for the fly phase///////
                            for(int i_motor = 9; i_motor < 12; i_motor++){
                                fly_q[i_motor] = All_joint[i_motor];
                                fly_dq[i_motor] = dq_joint[i_motor];
                                Kp[i_motor] = joint_Kp[i_motor];// + 30;
                                Kd[i_motor] = joint_Kv[i_motor];
                            }

                            // for ( int i_j = 9; i_j < 12; i_j++ ) {

                            //  original_torque[i_j] = torque_q[i_j] * (tanh(90*(t_norminal_st[3])));
                            //     torque[i_j] = original_torque[i_j];

                            // }
                            
                            if (t_norminal_st[3] < 0.5)
                            {
                                t_norminal_st[3] = t_norminal_st[3]+(t_now-t_pre[3])*step_ct_stouch;
                            }
                            else{
                                t_norminal_st[3] = t_norminal_st[3] + 1*(t_now-t_pre[3])*step_ct_stouch;
                            }
                            
                            Store[3] = t_now-t_pre[3];                 
                            t_pre[3]=t_now; 
                        }


                        

                        for(int i_leg = 0; i_leg < 4; i_leg++){
                            qDes[0 + 3 * i_leg] = q_joint[0 + 3 * i_leg];
                            qDes[1 + 3 * i_leg] = q_joint[1 + 3 * i_leg];
                            qDes[2 + 3 * i_leg] = q_joint[2 + 3 * i_leg];
                        }  

                    }

                        for(int i_leg = 0; i_leg < 4; i_leg++){
                            int check;
                            SendLowROS.motorCmd[0 + 3 * i_leg].q = qDes[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].dq = dq_joint[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].Kp = Kp[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].Kd = Kd[0 + 3 * i_leg];
                            SendLowROS.motorCmd[0 + 3 * i_leg].tau = -0.65f * legsgn(i_leg%2);

                            SendLowROS.motorCmd[1 + 3 * i_leg].q = qDes[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].dq = dq_joint[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].Kp = Kp[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].Kd = Kd[1 + 3 * i_leg];
                            SendLowROS.motorCmd[1 + 3 * i_leg].tau = 0.0f;

                            SendLowROS.motorCmd[2 + 3 * i_leg].q = qDes[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].dq = dq_joint[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].Kp = Kp[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].Kd = Kd[2 + 3 * i_leg];
                            SendLowROS.motorCmd[2 + 3 * i_leg].tau = 0.0f;
  
                        }

                    if (flag_st[0] == 2 && flag_st[1] == 2 && flag_st[2] == 2 && flag_st[3] == 2){

                    for(int i_leg = 0; i_leg < 2; i_leg++){

                                SendLowROS.motorCmd[1 + 3 * i_leg].tau = 1.0* transition_rate * (per_stouch_motor_total_fil[1 + 3 * (i_leg)]) + 0.0 * tao_thigh_pitch[i_leg];//+ flag_st[i_leg] * transition_rate * Feedforward_Error_sat[1 + 3 * i_leg] //(per_st_motor_total_fil[1 + 3 * 0])//ILC_torque[1 + 3 * i_leg]
                                SendLowROS.motorCmd[2 + 3 * i_leg].tau = 1.0* transition_rate * (per_stouch_motor_total_fil[2 + 3 * (i_leg)]); //+ flag_st[i_leg] * transition_rate * Feedforward_Error_sat[2 + 3 * i_leg]//(per_st_motor_total_fil[2 + 3 * 0])//ILC_torque[2 + 3 * i_leg]

                        }
                        for(int i_leg = 2; i_leg < 4; i_leg++){

                                SendLowROS.motorCmd[1 + 3 * i_leg].tau = 1.0* transition_rate * (per_stouch_motor_total_fil[1 + 3 * i_leg]  + Feedforward_Error_sat[1 + 3 * i_leg])+ 0.0 * tao_thigh_pitch[i_leg];// + flag_st[i_leg] * transition_rate * Feedforward_Error_sat[1 + 3 * i_leg]//(per_st_motor_total_fil[1 + 3 * 2]) replaced by ILC_torque[1 + 3 * i_leg] 
                                SendLowROS.motorCmd[2 + 3 * i_leg].tau = 1.0*  transition_rate * (per_stouch_motor_total_fil[2 + 3 * i_leg]  + Feedforward_Error_sat[2 + 3 * i_leg]);// + flag_st[i_leg] * transition_rate * Feedforward_Error_sat[2 + 3 * i_leg]//(per_st_motor_total_fil[2 + 3 * 2]) /ILC_torque[1 + 3 * i_leg] 

                        }    
                    }
                    
                    else{
                        for(int i_leg = 0; i_leg < 2; i_leg++){

                                SendLowROS.motorCmd[1 + 3 * i_leg].tau = 1.0* transition_rate * (per_st_motor_total_fil[1 + 3 * (i_leg)]) + 0.0 * tao_thigh_pitch[i_leg];//+ flag_st[i_leg] * transition_rate * Feedforward_Error_sat[1 + 3 * i_leg] //(per_st_motor_total_fil[1 + 3 * 0])//ILC_torque[1 + 3 * i_leg]
                                SendLowROS.motorCmd[2 + 3 * i_leg].tau = 1.0* transition_rate * (per_st_motor_total_fil[2 + 3 * (i_leg)]); //+ flag_st[i_leg] * transition_rate * Feedforward_Error_sat[2 + 3 * i_leg]//(per_st_motor_total_fil[2 + 3 * 0])//ILC_torque[2 + 3 * i_leg]

                        }
                        for(int i_leg = 2; i_leg < 4; i_leg++){

                                SendLowROS.motorCmd[1 + 3 * i_leg].tau = 1.0* transition_rate * (per_st_motor_total_fil[1 + 3 * i_leg]  + Feedforward_Error_sat[1 + 3 * i_leg])+ 0.0 * tao_thigh_pitch[i_leg];// + flag_st[i_leg] * transition_rate * Feedforward_Error_sat[1 + 3 * i_leg]//(per_st_motor_total_fil[1 + 3 * 2]) replaced by ILC_torque[1 + 3 * i_leg] 
                                SendLowROS.motorCmd[2 + 3 * i_leg].tau = 1.0*  transition_rate * (per_st_motor_total_fil[2 + 3 * i_leg]  + Feedforward_Error_sat[2 + 3 * i_leg]);// + flag_st[i_leg] * transition_rate * Feedforward_Error_sat[2 + 3 * i_leg]//(per_st_motor_total_fil[2 + 3 * 2]) /ILC_torque[1 + 3 * i_leg] 

                        }                    
                    }
std::cerr << "flag_st[0] = " << flag_st[0]  << std::endl;  
std::cerr << "transition_rate[3] = " << transition_rate  << std::endl;  
                                 std::cerr << "SendLowROS.motorCmd[2 + 3 * i_leg].tau[0][0][50] = " << SendLowROS.motorCmd[2].tau  << std::endl;    
                                 std::cerr << "SendLowROS.motorCmd[0 + 3 * i_leg].tau[0][0][50] = " << SendLowROS.motorCmd[0].tau  << std::endl;    
 std::cerr << "per_st_motor_total_fil= " << per_st_motor_total_fil[2]  << std::endl;   
                    
                    if(motiontime > 1){
                        // safe.PositionProtect(SendLowLCM, RecvLowLCM, 0.087);
                        // safe.PositionLimit(SendLowLCM);
                    }
                    
                }

            }

        }

        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);

        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();

        count++;
        if(count > 10){
            count = 10;
            initiated_flag = true;
            //safe.PowerProtect(SendLowLCM, RecvLowLCM, 10);// You can uncomment it for position protection
        }

        ros::Time end = ros::Time::now();
        ros::Duration d1 = end - begin;
        ROS_INFO("one control loop costs %lf milisecs", d1.toSec() * 1e3);
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "position_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    ros::NodeHandle nm;
    // JoyListener joy_listener;
    // sub_joy_msg = nm.subscribe("/joy", 100, &JoyListener::JoyData_Callback, &joy_listener);

    


    // ros::Rate loop_rate(1000);
    lowState_pub = nm.advertise<unitree_legged_msgs::LowState>("/" + firmwork + "/lowState/state", 100);
    lowCmd_pub = nm.advertise<unitree_legged_msgs::LowCmd>("/" + firmwork + "/lowCmd/cmd", 100);
    desired_pub = nm.advertise<std_msgs::Float32MultiArray>("/" + firmwork + "/lowState/desired", 100);

        Imu_pub = nm.advertise<unitree_legged_msgs::IMU>("/" + firmwork + "/lowState/imu",100);
        Logical_pub = nm.advertise<std_msgs::Int8>("/" + firmwork + "/jump_switch",1000);
        

    #ifdef SDK3_1
        aliengo::Control control(aliengo::LOWLEVEL);
        aliengo::LCM roslcm;
        mainHelper<aliengo::LowCmd, aliengo::LowState, aliengo::LCM>(argc, argv, roslcm);
    #endif

    #ifdef SDK3_2
        std::string robot_name;
        UNITREE_LEGGED_SDK::LeggedType rname;
        ros::param::get("/robot_name", robot_name);
        if(strcasecmp(robot_name.c_str(), "A1") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::A1;
        else if(strcasecmp(robot_name.c_str(), "Aliengo") == 0)
            rname = UNITREE_LEGGED_SDK::LeggedType::Aliengo;
            
        // UNITREE_LEGGED_SDK::Control control(rname, UNITREE_LEGGED_SDK::LOWLEVEL);
        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);

        mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif

    // ros::AsyncSpinner spinner(1);
    // spinner.start();
}