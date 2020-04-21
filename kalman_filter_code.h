/*
 *  kalman_filter_code.h
 *
 *  Created on: Apr 19, 2020
 *  Author: Muhammad Nauman Nasir
 */

#ifndef KALMAN_FILTER_CODE_H_
#define KALMAN_FILTER_CODE_H_

#include <iostream>
#include <string>
#include <cmath>

using namespace std;
/** \class KalmanFilter kalman_filter.h
 * \brief Class containing functionality of the Kalman filter
 *
 * The process is described with equation q(k) = A * q(k-1) + B * u(k-1)
 * The output is described with equation y(k) = C * q(k)
 * Just 1D example based on velocity data
 * Array containing velocity data coming from IMU arr[4] = {30, 32, 0 , 28}called as measurements
 * Steps:
 * Make state Matrix Equation
 * Calculate Process Covariance Matrix
 * Calculate Predicted Process Covariance Matrix P = P_old + Q ;
 * Calculate Kalman Gain   K = P / (P+R);
 * Calculate New observation from GPS, IMU etc
 * Calculate Current state X = X_pre + K (measurments - K X_pre)
 * Calculate Process Covariance Matrix P = (I - K H ) P_old
 */

// initial conditions
  static int A = 1;
  static int B = 0;
  static int Control_Input = 0;
  static int arr[4] = {30, 32, 0 , 28};

class KalmanFilterCode
{
public:
	/**
	 * Constructor
	 */
	KalmanFilterCode();
	/**
	  * Destructor
	  */
    ~KalmanFilterCode();

	static int prediction2d(int estimate);
	static int covariance2d(int sigma1);
	static float predicted_Process_Covmatrix(void);
	static float kalman_gain();
	static float Calculate_new_current_state();
	static float Update_new_Process_Cov_matrix();
};
#endif /* KALMAN_FILTER_CODE_H_ */
