#ifndef __FUSIONEKF_H__
#define __FUSIONEKF_H__

#include "measurement_package.h"
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF 
{
public:
	FusionEKF();
	~FusionEKF();

	/**
	* Run the whole flow of the Kalman Filter from here.
	*/
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);

	/**
	* Kalman Filter update and prediction math lives in here.
	*/
	KalmanFilter ekf;

private:
	Tools tools;

	// check whether the tracking toolbox was initialized or not (first measurement)
	bool is_initialized;

	long long previous_timestamp;

	double noise_ax;
	double noise_ay;

	Eigen::MatrixXd R_laser, R_radar;
	Eigen::MatrixXd H_laser, Hj;
};

#endif // __FUSIONEKF_H__ 
