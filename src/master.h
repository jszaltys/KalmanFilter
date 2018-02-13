#ifndef __MASTER_H__
#define __MASTER_H__

#include <uWS/uWS.h>
#include "libs.h"
#include "FusionEKF.h"
#include "measurement_package.h"
#include "json.hpp"


class Master
{
public:
	Master						 ();
	~Master						 ();

	int run						 ();
private:
	std::string hasData			 (const std::string &s);

	MeasurementPackage			 meas_package;
	Tools						 tools;
	FusionEKF					 fusionEKF;
	uWS::Hub					 h;

	std::vector<Eigen::VectorXd> estimations;
	std::vector<Eigen::VectorXd> ground_truth;
};


#endif // __MASTER_H__ 