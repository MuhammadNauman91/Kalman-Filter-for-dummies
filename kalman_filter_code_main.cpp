/*
 * kalman_filter_code_main.cpp
 *
 *  Created on: Apr 19, 2020
 *  Author: Muhammad Nauman Nasir
 */
#include<iostream>

#include "kalman_filter_code.h"
#include <array>


using namespace std;

int main()
{
 KalmanFilterCode kl;
 kl.prediction2d(10);
 kl.covariance2d(5);
 kl.predicted_Process_Covmatrix();
 kl.kalman_gain();
 kl.Calculate_new_current_state();
 kl.Update_new_Process_Cov_matrix();
 return 0;
}
