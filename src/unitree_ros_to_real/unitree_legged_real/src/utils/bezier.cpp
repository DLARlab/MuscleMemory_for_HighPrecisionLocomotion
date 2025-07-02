#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace std;

# include "pdflib.hpp"
# include "bezier.h"

float subterm_bez(int bez_para, int bez_k, float t_norminal)
{
     float subval;
     float bez_a;
     float bez_b;
     float bez_c;

     bez_para = bez_para-1;

     if (bez_k ==0 )
     {
         bez_a = (1 - t_norminal);
         bez_b = bez_para - bez_k;
         bez_c = pow(bez_a, bez_b);
         subval = r8_choose(bez_para, bez_k) * bez_c;

     }
        else if(bez_para == bez_k)
        {
             subval = r8_choose(bez_para, bez_k)*pow(t_norminal, bez_k);

        }
        else
        {
            bez_a = (1 - t_norminal);
            bez_b= bez_para - bez_k;
            bez_c=pow(bez_a, bez_b);
            subval=r8_choose(bez_para, bez_k)*pow(t_norminal, bez_k) * bez_c;

        }

     return subval;
}

float fcn_bezier(Eigen::MatrixXf coeff, float t_norminal)
{
     float fcn;
     int m;

     m = coeff.rows();
     fcn = 0;
     for (int bez_k = 0; bez_k < m; bez_k++) 
     {
        fcn =  fcn + coeff(bez_k, 0) * subterm_bez(m, bez_k, t_norminal);
     }

     return fcn;
}


Eigen::VectorXf diff_coeff(const Eigen::VectorXf& coeff_d) 
{
  int M = coeff_d.rows()-1;
  Eigen::MatrixXf A(M, M + 1);  // Dynamic size matrix
  Eigen::MatrixXf At(M + 1, M);
  Eigen::MatrixXf dcoefft(1, M);
  Eigen::MatrixXf dcoeff(M , 1);
     for (int i = 0; i < M; i++){
       for (int j = 0; j < M + 1; j++){
         A(i,j) = 0.0;
       }
     }

     for (int i = 0; i < M; i++)
     {
        A(i,i) = - (M-i) * r8_choose(M,i)/r8_choose(M-1, i);
        A(i,i+1) =  (i+1) * r8_choose(M,i+1)/r8_choose(M-1, i);
     }

     A(M-1, M) = M * r8_choose(M,M);

     At = A.transpose();
     dcoefft = coeff_d.transpose() * At;
     dcoeff = dcoefft.transpose();

     return  dcoeff;
}

float fcn_dbezier(Eigen::Matrix<float,6,1> coeff_d,float t_norminal)
{
  Eigen::Matrix<float,5,1> dcoeff;
  float dfcn;
  
  dfcn = 0.0;
  dcoeff = diff_coeff(coeff_d);
  dfcn = fcn_bezier(dcoeff, t_norminal);
  
  return dfcn;
}

float fcn_d2bezier(Eigen::VectorXf coeff_d,float t_norminal)
{
  Eigen::Matrix<float,5,1> dcoeff;
  Eigen::Matrix<float,4,1> d2coeff;
  float dfcn;
  float d2fcn;
  
  dfcn = 0.0;
  d2fcn = 0.0;
  dcoeff = diff_coeff(coeff_d);
  d2coeff = diff_coeff(dcoeff);
  d2fcn = fcn_bezier(d2coeff, t_norminal);
  
  return d2fcn;
}