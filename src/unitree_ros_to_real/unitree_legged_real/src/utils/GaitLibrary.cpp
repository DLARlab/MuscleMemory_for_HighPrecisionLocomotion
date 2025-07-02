#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include "GaitLibrary.h"
#include "Interpolation.h"

void gait_library(float (&bez_params_st)[21][72], float (&bez_params_fly)[21][72], float (&bez_params_stouch)[21][72], float (&duration_times_st)[21], float (&duration_times_fly)[21], float (&duration_times_stouch)[21])
{
    /////////stand_phase//load array/////

    std::ifstream infile_bez_st[21];//define a stream
    std::ifstream infile_time_st[21];//define a stream

    //////fly phase//////

	std::ifstream infile_bez_fly[21];//define a stream
    std::ifstream infile_time_fly[21];//define a stream

    //////touch down stance phase
    std::ifstream infile_bez_stouch[21];//define a stream
    std::ifstream infile_time_stouch[21];//define a stream

    //Extracting paremeters of different speeds
    for(int i_lib=0; i_lib<1; i_lib++){

            std::stringstream buff;
            buff.precision(1);
            buff.setf(std::ios::fixed);
            float order_038 = (i_lib * 1.0 - 10.0)/10.0;           
            buff<<order_038;
            
            std::string str;
            str = buff.str();

            std::stringstream filename_st, filename_fly, filename_stouch, filename_time_st, filename_time_fly, filename_time_stouch;
            filename_st << "/home/dlar/Gait_Library/035d_04h_04t/standOff.txt";
            filename_fly << "/home/dlar/Gait_Library/035d_04h_04t/air.txt";
            filename_stouch << "/home/dlar/Gait_Library/035d_04h_04t/standTouch.txt";

            filename_time_st << "/home/dlar/Gait_Library/035d_04h_04t/standOff_time.txt";
            filename_time_fly << "/home/dlar/Gait_Library/035d_04h_04t/air_time.txt";
            filename_time_stouch << "/home/dlar/Gait_Library/035d_04h_04t/standTouch_time.txt";


	        infile_bez_st[i_lib].open(filename_st.str());//open the file
            infile_bez_fly[i_lib].open(filename_fly.str());//open the file
            infile_bez_stouch[i_lib].open(filename_stouch.str());//open the file
            infile_time_st[i_lib].open(filename_time_st.str());//open the file
            infile_time_fly[i_lib].open(filename_time_fly.str());//open the file
            infile_time_stouch[i_lib].open(filename_time_stouch.str());//open the file

            ///////read phase duration
            infile_time_st[i_lib] >> duration_times_st[i_lib];
            infile_time_fly[i_lib] >> duration_times_fly[i_lib];
            infile_time_stouch[i_lib] >> duration_times_stouch[i_lib];
            
	        for (int i_bez = 0; i_bez < 72; i_bez++){
		        infile_bez_st[i_lib] >> bez_params_st[i_lib][i_bez];
                //read a value(jump to another value when meet a space or tab)
                infile_bez_fly[i_lib] >> bez_params_fly[i_lib][i_bez];
                //read a value(jump to another value when meet a space or tab)
                infile_bez_stouch[i_lib] >> bez_params_stouch[i_lib][i_bez];
                //read a value(jump to another value when meet a space or tab)
            }

	        infile_bez_st[i_lib].close();//close the file after read
            infile_bez_fly[i_lib].close();//close the file after read
            infile_bez_stouch[i_lib].close();//close the file after read
            infile_time_st[i_lib].close();
            infile_time_fly[i_lib].close();
            infile_time_stouch[i_lib].close();
    }

}

void gaitpara_interpolation(Eigen::Matrix<float, 12, 6>& coeff_st, Eigen::Matrix<float, 12, 6>& coeff_fly, Eigen::Matrix<float, 12, 6>& coeff_stouch, float& step_time_st, float& step_time_fly,float& step_time_stouch, float (&bez_params_st)[21][72], float (&bez_params_fly)[21][72], float (&bez_params_stouch)[21][72], float (&duration_times_st)[21], float (&duration_times_fly)[21], float (&duration_times_stouch)[21], std::vector<float> &x_velocity, float fly_sel_velocity, float (&desired_velocity_trunk)[3], long motiontime)
{
    const int NoGBP = 1;
    float ini_sel_velocity = desired_velocity_trunk[0];  ///initialize for later
    float sel_velocity;
    std::vector<float> y_bezier, y_bezier_fly, y_bezier_stouch, y_times, y_times_fly,y_times_stouch;

    /////to initialize the variables of velocity and bezier parameter
    std::vector<float> xVals_velocity, yVals_bezier, yVals_bezier_fly,yVals_bezier_stouch;

    ///according to the value from main function
    if (motiontime < 5000){
        sel_velocity = ini_sel_velocity;          
    }
    else if(motiontime < 20000){
        sel_velocity = desired_velocity_trunk[0];      
    }
    else{
        sel_velocity = fly_sel_velocity;      
    }
    ///////////select the bezier parameters based on the velocity
    for (int i_xVal = 1; i_xVal <= NoGBP; i_xVal++) {
        xVals_velocity.push_back((float) desired_velocity_trunk[0] );//sel_velocity
    }

    /////////interpolate//////////////////////------------------------


        /////////To insert the interpolated value into 12x6 matrix/////////
    for (int i_coeff = 0; i_coeff < 12; i_coeff++){
        for (int j_coeff = 0; j_coeff< 6; j_coeff++){
            coeff_st(i_coeff,j_coeff) = bez_params_st[0][i_coeff+12*j_coeff];
            coeff_fly(i_coeff,j_coeff) = bez_params_fly[0][i_coeff+12*j_coeff];
            coeff_stouch(i_coeff,j_coeff) = bez_params_stouch[0][i_coeff+12*j_coeff];
        }
    }

        y_times.clear();
        y_times_fly.clear(); ////initial
        y_times_stouch.clear(); ////initial

    for ( int j_v = 0 ; j_v<1; j_v++){
        y_times.push_back( duration_times_st[j_v]);
        y_times_fly.push_back( duration_times_fly[j_v] );
        y_times_stouch.push_back( duration_times_stouch[j_v] );
    }
    //////delete the interpolated value

    step_time_st = y_times[0];
    step_time_fly = y_times_fly[0];
    step_time_stouch = y_times_stouch[0];
}

void gait_control_gains(float (&joint_Kp)[12], float (&joint_Kv)[12], float (&joint_Kp_f)[12], float (&joint_Kv_f)[12],float (&joint_Kp_fd)[12], float (&joint_Kv_fd)[12], int (&flag_st)[4], int (&flag_fly)[4], float (&t_norminal_st)[4], float (&t_norminal_fly)[4], float PD_time, float transition_rate_temp_st, long motiontime)
{
    // if (motiontime < PD_time && motiontime > 5000){
    //     ////after stand up, common control gains//////


    //     for(int i_leg = 0; i_leg < 2; i_leg++){

    //         if ( flag_st[i_leg] == 1  )
    //         {
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 100.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 100.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 10.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 10.0f;
    //             }
    //             else{
    //                 joint_Kp[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 100.0f;// + 30.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 100.0f;// + 30.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 10.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 10.0f;                    
    //             }
    //         }
    //         else if (flag_st[i_leg] == 2){
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp[0 + 3 * i_leg] = 20.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 50.0f;


    //                 joint_Kv[0 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 3.0f;     
    //             }
    //             else{
    //                 joint_Kp[0 + 3 * i_leg] = 20.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 50.0f;


    //                 joint_Kv[0 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 3.0f;                        
    //             }
    //         }
    //         else if ( flag_fly[i_leg] == 1 ){
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp_f[0 + 3 * i_leg] = 70.0f;
    //                 joint_Kp_f[1 + 3 * i_leg] = 100.0f;
    //                 joint_Kp_f[2 + 3 * i_leg] = 110.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 5.0f;
    //             }
    //             else{
    //                 joint_Kp_f[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp_f[1 + 3 * i_leg] = 30.0f;
    //                 joint_Kp_f[2 + 3 * i_leg] = 35.0f;

    //                 joint_Kv_f[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv_f[1 + 3 * i_leg] = 5.0f;
    //                 joint_Kv_f[2 + 3 * i_leg] = 5.0f;                    
    //             }

    //         }            
    //     }


    //     for(int i_leg = 2; i_leg < 4; i_leg++){

    //         if ( flag_st[i_leg] == 1  )
    //         {
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 100.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 100.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 10.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 10.0f;
    //             }
    //             else{
    //                 joint_Kp[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 100.0f;// + 30.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 100.0f;// + 30.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 10.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 10.0f;                    
    //             }
    //         }
    //         else if (flag_st[i_leg] == 2){
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp[0 + 3 * i_leg] = 20.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 50.0f;


    //                 joint_Kv[0 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 3.0f;     
    //             }
    //             else{
    //                 joint_Kp[0 + 3 * i_leg] = 20.0f;
    //                 joint_Kp[1 + 3 * i_leg] = 50.0f;
    //                 joint_Kp[2 + 3 * i_leg] = 50.0f;


    //                 joint_Kv[0 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 3.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 3.0f;                        
    //             }
    //         }
    //         else if ( flag_fly[i_leg] == 1 ){
    //             if ( t_norminal_st[i_leg] < 0.5){
    //                 joint_Kp_f[0 + 3 * i_leg] = 70.0f;
    //                 joint_Kp_f[1 + 3 * i_leg] = 100.0f;
    //                 joint_Kp_f[2 + 3 * i_leg] = 110.0f;

    //                 joint_Kv[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[1 + 3 * i_leg] = 5.0f;
    //                 joint_Kv[2 + 3 * i_leg] = 5.0f;
    //             }
    //             else{
    //                 joint_Kp_f[0 + 3 * i_leg] = 50.0f;
    //                 joint_Kp_f[1 + 3 * i_leg] = 30.0f;
    //                 joint_Kp_f[2 + 3 * i_leg] = 35.0f;

    //                 joint_Kv_f[0 + 3 * i_leg] = 5.0f;
    //                 joint_Kv_f[1 + 3 * i_leg] = 5.0f;
    //                 joint_Kv_f[2 + 3 * i_leg] = 5.0f;                    
    //             }

    //         }            
    //     }
    // }
    // ///////at the beginning///////////
    // else{
        for(int i_leg = 0; i_leg < 4; i_leg++){
            joint_Kp[0 + 3 * i_leg] = 10.0f;
            joint_Kp[1 + 3 * i_leg] = 20.0f;
            joint_Kp[2 + 3 * i_leg] = 20.0f;

            joint_Kv[0 + 3 * i_leg] = 3.0f;
            joint_Kv[1 + 3 * i_leg] = 3.0f;
            joint_Kv[2 + 3 * i_leg] = 3.0f;

            joint_Kp_f[0 + 3 * i_leg] = 10.0f;
            joint_Kp_f[1 + 3 * i_leg] = 20.0f;
            joint_Kp_f[2 + 3 * i_leg] = 20.0f;

            joint_Kv_f[0 + 3 * i_leg] = 3.0f;
            joint_Kv_f[1 + 3 * i_leg] = 3.0f;
            joint_Kv_f[2 + 3 * i_leg] = 3.0f;

            joint_Kp_fd[0 + 3 * i_leg]= 50.0f*0.2;
            joint_Kp_fd[1 + 3 * i_leg]= 70.0f*0.2;
            joint_Kp_fd[2 + 3 * i_leg]= 70.0f*0.2;

            joint_Kv_fd[0 + 3 * i_leg] = 5.0f*0.1;
            joint_Kv_fd[1 + 3 * i_leg] = 5.0f*0.1;
            joint_Kv_fd[2 + 3 * i_leg] = 5.0f*0.1;
        }
    // }
        
}

