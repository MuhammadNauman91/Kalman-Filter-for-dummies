/*
 *  kalman_filter_code.cpp
 *
 *  Created on: Apr 19, 2020
 *  Author: Muhammad Nauman Nasir
 */

#include<iostream>
#include "kalman_filter_code.h"
#include <array>
#define Process_noise 0.001
#define Observation_error 0.01
using namespace std;

/**
 * Constructor
 */
KalmanFilterCode::KalmanFilterCode(){
}
/**
  * Destructor
  */
KalmanFilterCode::~KalmanFilterCode(){}
int KalmanFilterCode::prediction2d(int estimate)
{
    int System_State = A * (estimate) + B * (Control_Input);
    return System_State;
}
int KalmanFilterCode::covariance2d(int sigma1)
{
    int cov_matrix = pow(sigma1, 2);
    return cov_matrix;
}
float KalmanFilterCode::predicted_Process_Covmatrix(void)
{
	int sigma1 = 5;
    int pred_cov_matrix_old = KalmanFilterCode::covariance2d(sigma1);
    float pred_cov_matrix = pred_cov_matrix_old + Process_noise ;
    return pred_cov_matrix ;
}
float KalmanFilterCode::kalman_gain()
{
    float kal_gain = KalmanFilterCode:: predicted_Process_Covmatrix () / (KalmanFilterCode::predicted_Process_Covmatrix () + Observation_error);
    return kal_gain ;
}
float KalmanFilterCode::Calculate_new_current_state()
{
	int size = sizeof(arr)/sizeof(arr[0]);
	 std::cout<< size <<std::endl;
    for(int i = 0; i <= size; i++)
     {
    int measurement_readings = arr[i] + 0 ;
    float state_variable = 10;
    float j =  measurement_readings - KalmanFilterCode::prediction2d(state_variable) ;
    float P = kalman_gain() * j ;
    float New_current_state_X = KalmanFilterCode::prediction2d(state_variable) + P ;
    return New_current_state_X;
     }
}
float KalmanFilterCode::Update_new_Process_Cov_matrix()
{
    float UpdateProcssCov = (1 - KalmanFilterCode::kalman_gain()) * KalmanFilterCode::Calculate_new_current_state() ;
    return UpdateProcssCov;
}
