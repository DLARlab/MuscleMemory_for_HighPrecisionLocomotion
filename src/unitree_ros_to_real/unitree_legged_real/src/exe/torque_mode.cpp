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
# include "LowpassFilter.h"
# include "pdflib.hpp"
# include "rnglib.hpp"

#ifdef SDK3_1
using namespace aliengo;
#endif
#ifdef SDK3_2
using namespace UNITREE_LEGGED_SDK;
#endif

ros::Publisher desired_pub;
ros::Publisher lowCmd_pub;
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


    long motiontime=0;
    int rate_count = 0;
    int rate_count2 = 0;
    int rate_count3 = 0;
    int sin_count = 0;
    float qInit[12]={0};
    float qDes[12]={0};
    float sin_mid_q[12] = {0.0f, 0.96f, -2.0f, -0.0f, 0.96f, -2.0f, 0.0f, 0.96f, -2.0f, -0.0f, 0.96f, -2.0f};
    float Kp[12] = {0}; 
    float Kd[12] = {0};
    float torque[12] = {0}, torque_q[12];//torque_q is a intermediate value

    float All_joint[12] = {0},twist_trunk[6],All_joint_velfil[12], All_joint_vel[12], trunk_YPR[3], trunk_YPR_fil[3]; 
    float dq_joint[12] = {0};
    float joint_angle_des[12], joint_Kp[12], joint_Kv[12], joint_tor_compensation[12],motor_torque[12],motor_torque_pre[12],original_torque[12], pre_bezier[8], joint_Kp_f[12], joint_Kv_f[12],joint_Kp_fd[12],joint_Kv_fd[12];
    float L_leg_FR, L_leg_FL, L_leg_RR, L_leg_RL;
    float Kp_theta, Kd_theta, Kp_pitch, Kd_pitch, apha,beta, gama,vel_para, touchdelay, normaltime,tor_weight;
    float Error_q[12],st_sel_Error_q[12], Error_dq[12], q_joint[12] = {0}, per_st_motor_torque[12];
    
    float sin_joint[12];

    float param_hip_Kp, param_hip_Kd, param_thigh_Kp, param_thigh_Kd, param_calf_Kp, param_calf_Kd;

    int velfil_num = sizeof(All_joint_velfil)/sizeof(All_joint_velfil[0]);
    vel_para = 0.1;

    for(int i_ini = 0; i_ini < 12; i_ini++){     

      All_joint_velfil[i_ini] = 0.0;

    }

                            n.getParam("param_hip_Kp", param_hip_Kp);
                            n.getParam("param_hip_Kd", param_hip_Kd);
                            n.getParam("param_thigh_Kp", param_thigh_Kp);
                            n.getParam("param_thigh_Kd", param_thigh_Kd);
                            n.getParam("param_calf_Kp", param_calf_Kp);
                            n.getParam("param_calf_Kd", param_calf_Kd);

    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;
    std_msgs::Float32MultiArray desired_q_msg;

    UNITREE_LEGGED_SDK::Safety safe(LeggedType::A1);

    bool initiated_flag = false;  // initiate need time
    int count = 0;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
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
        printf("FL_1 position: %f\n",  RecvLowROS.motorState[FL_1].q);

            lowState_pub.publish(RecvLowROS);
            lowCmd_pub.publish(SendLowROS);                      
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
            
            desired_pub.publish(desired_q_msg);

        if(initiated_flag == true){
            motiontime++;
            std::cout << "Checking: the motiontime" <<motiontime<< std::endl;
            // gravity compensation
            SendLowROS.motorCmd[FR_0].tau = -0.65f;
            SendLowROS.motorCmd[FL_0].tau = +0.65f;
            SendLowROS.motorCmd[RR_0].tau = -0.65f;
            SendLowROS.motorCmd[RL_0].tau = +0.65f;
        

            if( motiontime >= 0){
                
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

                if( motiontime >= 10 && motiontime < 3000){
                    // printf("%f %f %f\n", );
                    rate_count++;
                    double rate = rate_count/2800.0;                       // needs count to 1500

                           for(int i_motor = 0; i_motor < 4; i_motor++){
                                Kp[0 + 3 * i_motor] = 55.0f;
                                Kp[1 + 3 * i_motor] = 90.0f;
                                Kp[2 + 3 * i_motor] = 100.0f;

                                Kd[0 + 3 * i_motor] = 3.0f;
                                Kd[1 + 3 * i_motor] = 3.0f;
                                Kd[2 + 3 * i_motor] = 3.0f;
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
                         SendLowROS.motorCmd[0 + 3 * i_leg].tau =  -0.65f * legsgn(i_leg%2);
                         check = legsgn(i_leg%2);
                         std::cout << "Checking: the direction" << check << std::endl;

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

                    sin_count++; 

                            cout << param_hip_Kp<< ";;;" << param_thigh_Kp << ";;;" <<param_calf_Kp <<endl;


                    if((param_hip_Kp) > 20 && (param_hip_Kd) > 1  && (param_thigh_Kp) > 20  && (param_thigh_Kd) > 2 && (param_calf_Kp) > 30  && (param_calf_Kd) > 2 && (param_hip_Kp) < 300 && (param_hip_Kd) < 50  && (param_thigh_Kp) < 300  && (param_thigh_Kd) < 50 && (param_calf_Kp) < 300  && (param_calf_Kd) < 50)
                    {
                           for(int i_motor = 0; i_motor < 4; i_motor++){

                                joint_Kp[0 + 3 * i_motor] = param_hip_Kp;
                                joint_Kp[1 + 3 * i_motor] = param_thigh_Kp;
                                joint_Kp[2 + 3 * i_motor] = param_calf_Kp;

                                joint_Kv[0 + 3 * i_motor] = param_hip_Kd;
                                joint_Kv[1 + 3 * i_motor] = param_thigh_Kd;
                                joint_Kv[2 + 3 * i_motor] = param_calf_Kd;

                                joint_Kp_f[0 + 3 * i_motor] = param_hip_Kp;
                                joint_Kp_f[1 + 3 * i_motor] = param_thigh_Kp;
                                joint_Kp_f[2 + 3 * i_motor] = param_calf_Kp;

                                joint_Kv_f[0 + 3 * i_motor] = param_hip_Kd;
                                joint_Kv_f[1 + 3 * i_motor] = param_thigh_Kd;
                                joint_Kv_f[2 + 3 * i_motor] = param_calf_Kd;
                            }
                    }
                    else{
                           for(int i_motor = 0; i_motor < 4; i_motor++){

                                joint_Kp[0 + 3 * i_motor] = 10.0f;
                                joint_Kp[1 + 3 * i_motor] = 6.0f;
                                joint_Kp[2 + 3 * i_motor] = 20.0f;

                                joint_Kv[0 + 3 * i_motor] = 0.0f;
                                joint_Kv[1 + 3 * i_motor] = 0.0f;
                                joint_Kv[2 + 3 * i_motor] = 0.0f;

                                joint_Kp_f[0 + 3 * i_motor] = 10.0f;
                                joint_Kp_f[1 + 3 * i_motor] = 6.0f;
                                joint_Kp_f[2 + 3 * i_motor] = 20.0f;

                                joint_Kv_f[0 + 3 * i_motor] = 0.0f;
                                joint_Kv_f[1 + 3 * i_motor] = 0.0f;
                                joint_Kv_f[2 + 3 * i_motor] = 0.0f;
                            }
                    }                  
                            std::cout << param_hip_Kp<< ";;;;;;" << param_thigh_Kp << ";;;;;;" <<param_calf_Kp << std::endl;

                    for(int i_leg = 0; i_leg < 4; i_leg++){
                       sin_joint[0 + 3 * i_leg] = 0.0f;
                       sin_joint[1 + 3 * i_leg] = 0.1 * sin(1 * 2 * sin_count/1000.0); 
                       sin_joint[2 + 3 * i_leg] = 0.1 * sin(1 * 2 * sin_count/1000.0); 
                    } 
               

                    for(int i_leg = 0; i_leg < 4; i_leg++){
                       q_joint[0 + 3 * i_leg] = sin_mid_q[0 + 3 * i_leg] + sin_joint[0 + 3 * i_leg];
                       q_joint[1 + 3 * i_leg] = sin_mid_q[1 + 3 * i_leg] + sin_joint[1 + 3 * i_leg];
                       q_joint[2 + 3 * i_leg] = sin_mid_q[2 + 3 * i_leg] + sin_joint[2 + 3 * i_leg];
                    } 

                    ////generate the error for torques////tracking error///
                    for(int i_motor = 0; i_motor < 12; i_motor++){
                        Error_q[i_motor] = q_joint[i_motor]- All_joint[i_motor];
                    }

                    for(int i_leg = 0; i_leg < 4; i_leg++){
                       dq_joint[0 + 3 * i_leg] = 0.0f;
                       dq_joint[1 + 3 * i_leg] = 0.1 * 2 * cos(1 * 2 * sin_count/1000.0);
                       dq_joint[2 + 3 * i_leg] = 0.1 * 2 * cos(1 * 2 * sin_count/1000.0);
                    } 

                    for(int e_j = 0; e_j < 12; e_j++){

                        Error_dq[e_j] = dq_joint[e_j] - All_joint_vel[e_j];
                    }

                    for ( int i_j = 0; i_j < 12; i_j++ ) torque_q[i_j] = joint_Kp_f[i_j] * Error_q[i_j] + joint_Kv_f[i_j] * Error_dq[i_j];

                    for ( int i_j = 0; i_j < 12; i_j++ ) {

                        torque[i_j] = torque_q[i_j] ;
                        
                    }

                            for(int i_motor = 0; i_motor < 4; i_motor++){
                                Kp[0 + 3 * i_motor] = 70.0f;
                                Kp[1 + 3 * i_motor] = 200.0f;
                                Kp[2 + 3 * i_motor] = 200.0f;

                                Kd[0 + 3 * i_motor] = 5.0f;
                                Kd[1 + 3 * i_motor] = 5.0f;
                                Kd[2 + 3 * i_motor] = 5.0f;
                            }

                    for(int i_leg = 0; i_leg < 4; i_leg++){
                         int check;
                         SendLowROS.motorCmd[0 + 3 * i_leg].q = q_joint[0 + 3 * i_leg];
                         SendLowROS.motorCmd[0 + 3 * i_leg].dq = dq_joint[0 + 3 * i_leg];
                         SendLowROS.motorCmd[0 + 3 * i_leg].Kp = Kp[0 + 3 * i_leg];
                         SendLowROS.motorCmd[0 + 3 * i_leg].Kd = Kd[0 + 3 * i_leg];
                         SendLowROS.motorCmd[0 + 3 * i_leg].tau =  -0.65f * legsgn(i_leg%2);
                         check = legsgn(i_leg%2);
                         std::cout << "Checking: the direction" << check << std::endl;

                         SendLowROS.motorCmd[1 + 3 * i_leg].q = q_joint[1 + 3 * i_leg];
                         SendLowROS.motorCmd[1 + 3 * i_leg].dq = dq_joint[1 + 3 * i_leg];
                         SendLowROS.motorCmd[1 + 3 * i_leg].Kp = Kp[1 + 3 * i_leg];
                         SendLowROS.motorCmd[1 + 3 * i_leg].Kd = Kd[1 + 3 * i_leg];
                         SendLowROS.motorCmd[1 + 3 * i_leg].tau = 0.0f;

                         SendLowROS.motorCmd[2 + 3 * i_leg].q = q_joint[2 + 3 * i_leg];
                         SendLowROS.motorCmd[2 + 3 * i_leg].dq = dq_joint[2 + 3 * i_leg];
                         SendLowROS.motorCmd[2 + 3 * i_leg].Kp = Kp[2 + 3 * i_leg];
                         SendLowROS.motorCmd[2 + 3 * i_leg].Kd = Kd[2 + 3 * i_leg];
                         SendLowROS.motorCmd[2 + 3 * i_leg].tau = 0.0f;
                        std::cout << "Checking: the checking" << std::endl;
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
            safe.PositionProtect(SendLowLCM, RecvLowLCM, 0.087);
            safe.PowerProtect(SendLowLCM, RecvLowLCM, 10);// You can uncomment it for position protection
            safe.PositionLimit(SendLowLCM);
        }

        ros::Time end = ros::Time::now();
        ros::Duration d1 = end - begin;
        ROS_INFO("one control loop costs %lf milisecs", d1.toSec() * 1e3); 
               
    }    
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_ros_mode");
    std::string firmwork;
    ros::param::get("/firmwork", firmwork);

    ros::NodeHandle nm;

    lowState_pub = nm.advertise<unitree_legged_msgs::LowState>("/" + firmwork + "/lowState/state", 100);
    lowCmd_pub = nm.advertise<unitree_legged_msgs::LowCmd>("/" + firmwork + "/lowCmd/cmd", 100);
    desired_pub = nm.advertise<std_msgs::Float32MultiArray>("/" + firmwork + "/lowState/desired", 100);

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
            
        // UNITREE_LEGGED_SDK::InitEnvironment();
        UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
        
        mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
    #endif
}