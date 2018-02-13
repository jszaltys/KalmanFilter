#include "FusionEKF.h"


FusionEKF::FusionEKF() : is_initialized(false), previous_timestamp(0),noise_ax(9.0),noise_ay(9.0), R_laser(Eigen::MatrixXd::Zero(2, 2)), R_radar(Eigen::MatrixXd::Zero(3, 3)),
																								   H_laser(Eigen::MatrixXd::Zero(2, 4)), Hj(Eigen::MatrixXd::Zero(3, 4))
{
	R_laser(0, 0) = 0.0225;
	R_laser(1, 1) = 0.0225;

	R_radar(0, 0) = 0.09;
	R_radar(1, 1) = 0.0009;
	R_radar(2, 2) = 0.09;

	H_laser(0, 0) = 1.0;
	H_laser(1, 1) = 1.0;
}
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
	if (!is_initialized) 
	{
		ekf.init(Eigen::Vector4d(), Eigen::MatrixXd(4, 4), Eigen::MatrixXd(4, 4), Eigen::MatrixXd(4, 4));
	
		if (measurement_pack.sensor_type == MeasurementPackage::RADAR)
			ekf.x = tools.Polar2Cartesian_2D(measurement_pack.raw_measurements);

		else if (measurement_pack.sensor_type == MeasurementPackage::LASER)
			ekf.x << measurement_pack.raw_measurements[0], measurement_pack.raw_measurements[1], 0, 0;
		

		previous_timestamp = measurement_pack.timestamp;
		
		if (std::fabs(ekf.x(0)) < 0.0001 && std::fabs(ekf.x(1)) < 0.0001)
		{
			ekf.x(0) = 0.0001;
			ekf.x(1) = 0.0001;
		}
		
		is_initialized = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/

	const double dt = (measurement_pack.timestamp - previous_timestamp) / 1000000.0;
	previous_timestamp = measurement_pack.timestamp;

	ekf.F(0, 2) = dt;
	ekf.F(1, 3) = dt;

	const double dt_4 = std::pow(dt, 4.0) / 4.0;
	const double dt_3 = std::pow(dt, 3.0) / 2.0;
	const double dt_2 = std::pow(dt, 2.0);


	ekf.Q(0, 0) = dt_4*noise_ax;
	ekf.Q(0, 2) = dt_3*noise_ax;
	ekf.Q(1, 1) = dt_4*noise_ay;
	ekf.Q(1, 3) = dt_3*noise_ay;
	ekf.Q(2, 2) = dt_2*noise_ax;
	ekf.Q(3, 3) = dt_2*noise_ay;

	ekf.Q(3, 1) = ekf.Q(1, 3);
	ekf.Q(2, 0) = ekf.Q(0, 2);

	ekf.predict();


	/*****************************************************************************
	*  Update
	****************************************************************************/
	
	if (measurement_pack.sensor_type == MeasurementPackage::RADAR) 
	{
		ekf.H = tools.CalculateJacobian(ekf.x);
		ekf.R = R_radar;
		ekf.updateEKF(measurement_pack.raw_measurements);
	}
	else 
	{
		ekf.H = H_laser;
		ekf.R = R_laser;
		ekf.updateKF(measurement_pack.raw_measurements);	
	}
	
	std::cout << "ESTIMATES: "<< std::endl;
	std::cout << "x = " << ekf.x(0) <<"  "<<"y = "<< ekf.x(0) << std::endl<<std::endl;
	std::cout << "P = " << ekf.P << std::endl;
}
