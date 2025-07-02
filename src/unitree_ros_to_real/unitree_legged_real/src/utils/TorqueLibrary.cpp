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
#include "textfileparser.h"

void torque_record(float (&torque_stance)[300][12], float (&Error_stance)[300][12], float (&dError_stance)[300][12], float (&Pre_Error_stance)[300][12], float (&Pre_dError_stance)[300][12], float (&Feedforward_Error_tor_stance)[300][12], float (&pre_torque_stance_fil_zero)[300][12], float (&pre_torque_stance_fil_err_tor)[300][12], float (&total_stance_torque_fil)[300][12], float (&pre_torque_stance_fil_total)[300][12], int stride_counter)
{
    float ILC_Switch[2];

    kb::math::FilterCoefficients<double> fc{ 
    m_CoefficientsA:{1.0000,-2.374094743709352,1.929355669091215,-0.532075368312092}, 
    m_CoefficientsB:{2.898194633721429e-03,8.694583901164288e-03,8.694583901164288e-03,2.898194633721429e-03}
    };

    kb::math::Filt<double> filt(fc);
    kb::math::FiltFilt<double> filtfilt(fc);
    if (stride_counter > 1){
        ////in the first 5 strides, the system will direct record the raw torque data(related to the first order filter)
        if (stride_counter > 4 && stride_counter < 10){
             ILC_Switch[0] = 0;
             ILC_Switch[1] = 1/0.1;          // this value associate with the gains in first order filter belew.
        }
        else{
             ILC_Switch[0] = 1;
             ILC_Switch[1] = 1;
        }
        

        for (int i_st = 0 ; i_st < 300 ; i_st++){
            for ( int j_st = 0 ; j_st<12; j_st++){
                Pre_Error_stance[i_st][j_st] = Error_stance[i_st][j_st];
                Pre_dError_stance[i_st][j_st] = dError_stance[i_st][j_st];
                //  cout  << Pre_torque_stance[0][0] << ","<< " torque_stance" << torque_stance[0][0]<< ",";           
                }
              }

        for ( int j_st = 0 ; j_st<12; j_st++){
            std::vector<double> temp_vec;
            std::vector<double> temp_vec_error;
            temp_vec.reserve(300);
            temp_vec_error.reserve(300);

            ///per_st_motor_torque_fil_unshift[j_st] = Pre_torque_stance[0][j_st]; 

            for ( int i_st = 0; i_st < 300; i_st++ ){                                     
                temp_vec.push_back(torque_stance[i_st][j_st]);
                temp_vec_error.push_back(Feedforward_Error_tor_stance[i_st][j_st]);
            }

            auto zeroPhaseFiltered  = filtfilt.ZeroPhaseFiltering(temp_vec); 
            auto zeroPhaseFiltered_err_tor  = filtfilt.ZeroPhaseFiltering(temp_vec_error);

            for ( int i_st = 0; i_st < 300; i_st++ ){
                pre_torque_stance_fil_zero[i_st][j_st] = zeroPhaseFiltered[i_st];
                pre_torque_stance_fil_err_tor[i_st][j_st] = zeroPhaseFiltered_err_tor[i_st];

                total_stance_torque_fil[i_st][j_st] = 0.9 * ILC_Switch[0] * pre_torque_stance_fil_total[i_st][j_st] +  0.1 * ILC_Switch[1] * pre_torque_stance_fil_zero[i_st][j_st];
                pre_torque_stance_fil_total[i_st][j_st] = total_stance_torque_fil[i_st][j_st];
            }

            for (int i_st = 0 ; i_st < 300 ; i_st++){
                for ( int j_st = 0 ; j_st<12; j_st++){
                    Error_stance[i_st][j_st] = 0.0;
                    dError_stance[i_st][j_st] = 0.0;
                }                 
            }             

        }              
    }

}

void torque_library(float (&Torque_Profile_st)[12][21][230])
{
    /////////**********stand_phase////////////
	        //load array //define a place to store

    // float Torque_Profile_st_FRThigh[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_FRCalf[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_FLThigh[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_FLCalf[21][230] = { 0.0 };//define a place to store toruqe

    // float Torque_Profile_st_RRThigh[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_RRCalf[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_RLThigh[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st_RLCalf[21][230] = { 0.0 };//define a place to store toruqe
    // float Torque_Profile_st[12][21][230] = { 0.0 };//define a place to store toruqe

    // std::ifstream infile_Torque_st_FRThigh[21];//define a stream
    // std::ifstream infile_Torque_st_FRCalf[21];//define a stream
    // std::ifstream infile_Torque_st_FLThigh[21];//define a stream
    // std::ifstream infile_Torque_st_FLCalf[21];//define a stream

    // std::ifstream infile_Torque_st_RRThigh[21];//define a stream
    // std::ifstream infile_Torque_st_RRCalf[21];//define a stream
    // std::ifstream infile_Torque_st_RLThigh[21];//define a stream
    // std::ifstream infile_Torque_st_RLCalf[21];//define a stream
    std::ifstream infile_Torque_st[12][21];//define a stream

    //Extracting paremeters of different speeds
    for(int i_lib=0; i_lib<21; i_lib++){
        std::stringstream buff;
            buff.precision(1);
            buff.setf(std::ios::fixed); 
            float order_038 = (i_lib * 1.0 - 10.0)/10.0;   //convert issue    
            buff<<order_038;
           
            std::string str;
            str = buff.str();
            /////////////////////////for torque/////////////////
            std::stringstream filename_torque[12];

            for (int j_lib=0; j_lib<12; j_lib++){
                std::stringstream buff2;
                int order = j_lib + 1;                
                ///////// convert int to str ///////
                buff2<<order;
                std::string str2;
                str2 = buff2.str();

                filename_torque[j_lib] << "/home/dlar/Gait_Library/Torquelib0723/" << str2 << "th_Torque_v"<< str << ".txt";
                infile_Torque_st[j_lib][i_lib].open(filename_torque[j_lib].str());//open the file

               //debugging tool//j_lib=11//////////////
                // if (i_lib == 14 && j_lib == 11)
                // {
                //   std::ofstream file1("Torque_Profile_st1204.txt");
                //     file1 << filename_torque[j_lib].str() << "\n" << str <<"\n"<< order_038 <<"\n";
                    
                //     file1.close();  
                // }

                for (int i_sample = 0; i_sample < 230; i_sample++){

                    infile_Torque_st[j_lib][i_lib] >> Torque_Profile_st[j_lib][i_lib][i_sample];//read a value(jump to another value when meet a space or tab)
                }
                infile_Torque_st[j_lib][i_lib].close();//close the file after read
            }

            //debugging tool//////////////////////////////
            // std::ofstream file("Torque_Profile_st.txt");
            //             for (int k = 0; k < 230; ++k) {
            //                 file << Torque_Profile_st[11][14][k];
            //                 if (k != 229) file << " ";  // Separate values with spaces
            //             }
            // file.close(); 

            //////////// For Front right leg ////////////////

            // std::stringstream filename_torque_FRthigh;
            // filename_torque_FRthigh << "/home/dlar/Gait_Library/Torquelib0723/Torque_Thigh_v"<< str << ".txt";
	        // infile_Torque_st_FRThigh[i_lib].open(filename_torque_FRthigh.str());//open the file

	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    //     infile_Torque_st_FRThigh[i_lib] >> Torque_Profile_st_FRThigh[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_FRThigh[i_lib].close();//close the file after read

            // std::stringstream filename_torque_FRcalf;
            // filename_torque_FRcalf << "/home/dlar/Gait_Library/Torquelib0723/Torque_Calf_v"<< str << ".txt";
	        // infile_Torque_st_FRCalf[i_lib].open(filename_torque_FRcalf.str());//open the file
	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    // infile_Torque_st_FRCalf[i_lib] >> Torque_Profile_st_FRCalf[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_FRCalf[i_lib].close();//close the file after read   

            // //////////// For Front left leg ////////////////

            // std::stringstream filename_torque_FLthigh;
            // filename_torque_FLthigh << "/home/dlar/Gait_Library/Torquelib0723/Torque_Thigh_v"<< str << ".txt";
	        // infile_Torque_st_FLThigh[i_lib].open(filename_torque_FLthigh.str());//open the file

	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    //     infile_Torque_st_FLThigh[i_lib] >> Torque_Profile_st_FLThigh[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_FLThigh[i_lib].close();//close the file after read

            // std::stringstream filename_torque_FLcalf;
            // filename_torque_FLcalf << "/home/dlar/Gait_Library/Torquelib0723/Torque_Calf_v"<< str << ".txt";
	        // infile_Torque_st_FLCalf[i_lib].open(filename_torque_FLcalf.str());//open the file
	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    // infile_Torque_st_FLCalf[i_lib] >> Torque_Profile_st_FLCalf[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_FLCalf[i_lib].close();//close the file after read   

            // //////////// For Rear Right leg ////////////////

            // std::stringstream filename_torque_RRthigh;
            // filename_torque_RRthigh << "/home/dlar/Gait_Library/Torquelib0723/Torque_Thigh_v"<< str << ".txt";
	        // infile_Torque_st_RRThigh[i_lib].open(filename_torque_RRthigh.str());//open the file

	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    //     infile_Torque_st_RRThigh[i_lib] >> Torque_Profile_st_RRThigh[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_RRThigh[i_lib].close();//close the file after read

            // std::stringstream filename_torque_RRcalf;
            // filename_torque_RRcalf << "/home/dlar/Gait_Library/Torquelib0723/Torque_Calf_v"<< str << ".txt";
	        // infile_Torque_st_RRCalf[i_lib].open(filename_torque_RRcalf.str());//open the file
	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    // infile_Torque_st_RRCalf[i_lib] >> Torque_Profile_st_RRCalf[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_RRCalf[i_lib].close();//close the file after read

            // //////////// For Rear left leg ////////////////

            // std::stringstream filename_torque_RLthigh;
            // filename_torque_RLthigh << "/home/dlar/Gait_Library/Torquelib0723/Torque_Thigh_v"<< str << ".txt";
	        // infile_Torque_st_RLThigh[i_lib].open(filename_torque_RLthigh.str());//open the file

	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    //     infile_Torque_st_RLThigh[i_lib] >> Torque_Profile_st_RLThigh[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_RLThigh[i_lib].close();//close the file after read

            // std::stringstream filename_torque_RLcalf;
            // filename_torque_RLcalf << "/home/dlar/Gait_Library/Torquelib0723/Torque_Calf_v"<< str << ".txt";
	        // infile_Torque_st_RLCalf[i_lib].open(filename_torque_RLcalf.str());//open the file
	        // for (int i_bez = 0; i_bez < 230; i_bez++){

		    // infile_Torque_st_RLCalf[i_lib] >> Torque_Profile_st_RLCalf[i_lib][i_bez];//read a value(jump to another value when meet a space or tab)
            // }
	        // infile_Torque_st_RLCalf[i_lib].close();//close the file after read
            //////////////////////////////////////////////////////////////////////////////////////
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


void loadFootForceData(float (&Footforce_con)[21][12][21]){

              //////////////////////////////////////////////////////////////////////////////////
              for(int i_lib=0; i_lib<21; i_lib++){
                 std::stringstream buff;
                 buff.precision(1);
                 buff.setf(std::ios::fixed); 
                 float order_038 = (i_lib - 10.0)/10.0;                
                 buff<<order_038;
           
                 std::string str;
                 str = buff.str();
                  std::string filename_frForce;
                  filename_frForce = "/home/dlar/Gait_Library/Footforce/standfFRfoot_H_0.34_V_" + str + "_FRFRRRRLlo_.txt";
                  std::vector<std::vector<float>> frForceVector = readTxtFileTo2DVector(filename_frForce);
                  for (const auto& row : frForceVector) {
                      for (const float& value : row) {
                        std::cout << value << " ";
                      }
                      std::cout << "\n";
                  }
                  std::string filename_flForce;
                  filename_flForce = "/home/dlar/Gait_Library/Footforce/standfFLfoot_H_0.34_V_" + str + "_FRFRRRRLlo_.txt";
                  std::vector<std::vector<float>> flForceVector = readTxtFileTo2DVector(filename_flForce);
                  if (flForceVector.empty()) {
                      std::cerr << "Error: Failed to read the file or the file is empty." << std::endl;
                  }
   
                  // Set precision to 1 decimal place
                  std::cout << std::fixed << std::setprecision(1);
                  // Access the values in the 2D vector and display them
                  for (const auto& row : flForceVector) {
                      for (const float& value : row) {
                        std::cout << value << " ";
                      }
                      std::cout << "\n";
                  }
    
                  std::string filename_rrForce;
                  filename_rrForce = "/home/dlar/Gait_Library/Footforce/standfRRfoot_H_0.34_V_" + str + "_FRFRRRRLlo_.txt";
                  std::vector<std::vector<float>> rrForceVector = readTxtFileTo2DVector(filename_rrForce);
                  for (const auto& row : rrForceVector) {
                      for (const float& value : row) {
                        std::cout << value << " ";
                      }
                      std::cout << "\n";
                  }
                  std::string filename_rlForce;
                  filename_rlForce = "/home/dlar/Gait_Library/Footforce/standfRLfoot_H_0.34_V_" + str + "_FRFRRRRLlo_.txt";
  
                  std::vector<std::vector<float>> rlForceVector = readTxtFileTo2DVector(filename_rlForce);
                  for (const auto& row : rlForceVector) {
                      for (const float& value : row) {
                        std::cout << value << " ";
                      }
                      std::cout << "\n";
                  }
                  for(int i_joint=0; i_joint<3; i_joint++){
                    for (int i_data=0; i_data<21; i_data++)
                    {
                      Footforce_con[i_lib][i_joint][i_data] = frForceVector[i_joint][i_data];
                    }
                  }
                  for(int i_joint=3; i_joint<6; i_joint++){
                    for (int i_data=0; i_data<21; i_data++)
                    {
                      Footforce_con[i_lib][i_joint][i_data] = flForceVector[i_joint-3][i_data];
                    }
                  }
                  for(int i_joint=6; i_joint<9; i_joint++){
                    for (int i_data=0; i_data<21; i_data++)
                    {
                      Footforce_con[i_lib][i_joint][i_data] = rrForceVector[i_joint-6][i_data];
                    }
                  }
                  for(int i_joint=9; i_joint<12; i_joint++){
                    for (int i_data=0; i_data<21; i_data++)
                    {
                      Footforce_con[i_lib][i_joint][i_data] = rlForceVector[i_joint-9][i_data];
                    }
                  }
   
              }
    
}

void loadDdxData(float (&ddx_con)[21][3][21]){

              //////////////////////////////////////////////////////////////////////////////////
              for(int i_lib=0; i_lib<21; i_lib++){
                 std::stringstream buff;
                 buff.precision(1);
                 buff.setf(std::ios::fixed); 
                 float order_038 = (i_lib - 10.0)/10.0;                
                 buff<<order_038;
           
                 std::string str;
                 str = buff.str();
                  std::string filename_ddx;
                  filename_ddx = "/home/dlar/Gait_Library/ddx/standAcc_H_0.34_V_" + str + "_FRFRRRRLlo_.txt";
                  std::vector<std::vector<float>> accVector = readTxtFileTo2DVector(filename_ddx);
                  for (const auto& row : accVector) {
                      for (const float& value : row) {
                        std::cout << value << " ";
                      }
                      std::cout << "\n";
                  }
                  
                  for(int i_joint=0; i_joint<3; i_joint++){
                    for (int i_data=0; i_data<21; i_data++)
                    {
                      ddx_con[i_lib][i_joint][i_data] = accVector[i_joint][i_data];
                    }
                  }
              }
    
}