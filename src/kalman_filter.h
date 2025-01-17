#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "Eigen/Dense"
#include "tools.h"

struct KalmanFilter
{
	KalmanFilter();
	~KalmanFilter();

	/**
	* Init Initializes Kalman filter
	* @param x_in Initial state
	* @param P_in Initial state covariance
	* @param F_in Transition matrix
	* @param H_in Measurement matrix
	* @param R_in Measurement covariance matrix
	* @param Q_in Process covariance matrix
	*/
	void init(const Eigen::VectorXd &x_in, const Eigen::MatrixXd &P_in, const Eigen::MatrixXd &F_in, const Eigen::MatrixXd &Q_in);

	/**
	* Prediction Predicts the state and the state covariance
	* using the process model
	* @param delta_T Time between k and k+1 in s
	*/
	void predict();

	/**
	* Updates the state by using standard Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void updateKF(const Eigen::VectorXd &z);

	/**
	* Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void updateEKF(const Eigen::VectorXd &z);


	Eigen::VectorXd x;
	Eigen::MatrixXd P, F, Q, H, R;

private:
	void update(const Eigen::VectorXd &z);
	
	
	Tools tools;
	const double pi = 3.141592653589793238462643383279502884197169399;
};

#endif // __KALMAN_FILTER_H__ 
