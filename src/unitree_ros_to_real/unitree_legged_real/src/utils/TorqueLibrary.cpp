#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "Interpolation.h"
#include "FiltFilt.h"

#include "TorqueLibrary.h"

void torque_record(float* Torque_Profile_st, float (*torque)[12], float (*Error)[12], float (*dError)[12], float (*Pre_Error)[12], float (*Pre_dError)[12], float (*Feedforward_Error_tor)[12], float (*pre_torque_fil_zero)[12], float (*pre_torque_fil_err_tor)[12], float (*total_torque_fil)[12], float (*pre_torque_fil_total)[12], int rows, int stride_counter)
{
    float ILC_Switch[2];

    kb::math::FilterCoefficients<double> fc{ 
    m_CoefficientsA:{1.0000,-2.374094743709352,1.929355669091215,-0.532075368312092}, 
    m_CoefficientsB:{2.898194633721429e-03,8.694583901164288e-03,8.694583901164288e-03,2.898194633721429e-03}
    };

    kb::math::Filt<double> filt(fc);
    kb::math::FiltFilt<double> filtfilt(fc);

    ILC_Switch[0] = 1;
    ILC_Switch[1] = 1;

    for (int i_st = 0 ; i_st < rows ; i_st++){
        for (int j_st = 0 ; j_st < 12 ; j_st++){
            Pre_Error[i_st][j_st] = Error[i_st][j_st];
            Pre_dError[i_st][j_st] = dError[i_st][j_st];
        }
    }

    for (int j_st = 0 ; j_st < 12 ; j_st++){
        std::vector<double> temp_vec;
        std::vector<double> temp_vec_error;
        temp_vec.reserve(rows);
        temp_vec_error.reserve(rows);

        for (int i_st = 0; i_st < rows; i_st++){                                     
            temp_vec.push_back((Torque_Profile_st)[j_st*1*rows+0*rows+i_st]);
            temp_vec_error.push_back(Feedforward_Error_tor[i_st][j_st]);
        }

        auto zeroPhaseFiltered = filtfilt.ZeroPhaseFiltering(temp_vec); 
        auto zeroPhaseFiltered_err_tor = filtfilt.ZeroPhaseFiltering(temp_vec_error);

        for (int i_st = 0; i_st < rows; i_st++){
            pre_torque_fil_zero[i_st][j_st] = zeroPhaseFiltered[i_st];
            pre_torque_fil_err_tor[i_st][j_st] = zeroPhaseFiltered_err_tor[i_st];

            total_torque_fil[i_st][j_st] = 0.0 * ILC_Switch[0] * pre_torque_fil_total[i_st][j_st] +  1.0 * ILC_Switch[1] * pre_torque_fil_zero[i_st][j_st];
            pre_torque_fil_total[i_st][j_st] = total_torque_fil[i_st][j_st];
        }

        for (int i_st = 0 ; i_st < rows ; i_st++){
            for (int j_st = 0 ; j_st < 12 ; j_st++){
                Error[i_st][j_st] = 0.0;
                dError[i_st][j_st] = 0.0;
            }                 
        }
    }
}


void torque_library(float* Torque_Profile_st, int size)
{
    // Define a stream for each file to be opened
    std::ifstream infile_Torque_st[12][21];

    // Extracting parameters for different speeds
    for (int i_lib = 0; i_lib < 1; i_lib++) {
        std::stringstream buff;
        buff.precision(1);
        buff.setf(std::ios::fixed); 
        float order_038 = (i_lib * 10.0 - 10.0) / 10.0;   // Convert issue    
        buff << order_038;
        
        std::string str, str2;
        str = buff.str();

        if (size == 439) {
            str = "st";
        }
        else {
            str = "stouch";
        }
        
        // For torque
        std::stringstream filename_torque[12];

        for (int j_lib = 0; j_lib < 12; j_lib++) {
            std::stringstream buff2;
            int order = j_lib + 1;                
            // Convert int to str
            buff2 << order;
            std::string str2;
            str2 = buff2.str();

            filename_torque[j_lib] << "/home/dlar/Gait_Library/035d_04h_04t/5Jump/Jump_" << str <<  str2 <<".txt";
            infile_Torque_st[j_lib][i_lib].open(filename_torque[j_lib].str()); // Open the file

            for (int i_sample = 0; i_sample < size; i_sample++) {
                infile_Torque_st[j_lib][i_lib] >> Torque_Profile_st[(j_lib * 1 + i_lib) * size + i_sample]; // Read a value
                
            }
            infile_Torque_st[j_lib][i_lib].close(); // Close the file after reading

        }
    }
}


// This is the function
void Torqueprofile_interpolation(float (&Torque_Profile_st)[12][21][230],float (&ILC_torque)[12], std::vector<float> &x_velocity, float fly_sel_velocity, float (&desired_velocity_trunk)[3], int int_pre_number, long motiontime)
{
    //////////////////////////////interpolate torque///////////////////////////////////////////
    ////initial
    std::vector<std::vector<float>> Torque_temp_st(12, std::vector<float>(21)); 
 
        for ( int j_v = 0 ; j_v < 12; j_v++){
            for ( int i_v = 0 ; i_v < 21; i_v++){
                    Torque_temp_st[j_v][i_v] = Torque_Profile_st[j_v][i_v][int_pre_number];
            }
        }

        // // //debugging tool//Print the 2D vector
        // for (int i = 0; i < 12; i++) {
        //     for (int j = 0; j < 21; j++) {
        //         std::cout << Torque_temp_st[i][j] << " ";
        //     }
        //     std::cout << std::endl;
        // }

        // std::ofstream file("Torque_temp_st.txt");

        // for (const auto& row : Torque_temp_st) {
        //     for (size_t i = 0; i < row.size(); ++i) {
        //       file << row[i];
        //       if (i != row.size() - 1) file << " ";  // Separate values with spaces
        //     }
        //     file << "\n";  // New line for each row
        // }

        // file.close();
        const int NoGBP = 1;
        float ini_sel_velocity = desired_velocity_trunk[0];  ///initialize for later
        float sel_velocity;
        float Torque_into_st[12];

        /////to initialize the variables of velocity and bezier parameter
        std::vector<float> xVals_velocity;

        ///according to the value from main function
        if (motiontime < 7000){
            sel_velocity = desired_velocity_trunk[0];          
        }
        else if(motiontime < 200000){
            sel_velocity = desired_velocity_trunk[0];      
        }
        else{
            sel_velocity = fly_sel_velocity;      
        }
        ///////////select the bezier parameters based on the velocity
        for (int i_xVal = 1; i_xVal <= NoGBP; i_xVal++) {
            xVals_velocity.push_back((float) desired_velocity_trunk[0] );//sel_velocity
        }

        for (float inter_x_velocity : xVals_velocity)
        {
            for ( int j_v = 0 ; j_v < 12; j_v++){
                Torque_into_st[j_v] = linear_interpolate( x_velocity, Torque_temp_st[j_v], inter_x_velocity, true);
            }


        //debugging tool//////////////////////////////
        // std::ofstream file("Torque_into_st.txt");

        // for (size_t i = 0; i < 12; ++i) {
        //     file << Torque_into_st[i];
        //     if (i != 12 - 1) file << " ";  // Separate values with spaces
        // }

        // file.close();            

            for ( int j_v = 0 ; j_v < 12; j_v++)
            { 
                if (int_pre_number <=230){

                    ILC_torque[j_v] = Torque_into_st[j_v];
                }
                else{

                    ILC_torque[j_v] = Torque_Profile_st[j_v][0][230];  ///need to discuss later 07/20/23
            
                }
            }

        }

}
