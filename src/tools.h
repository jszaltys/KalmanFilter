#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <vector>
#include <iostream>
#include "Eigen/Eigen"

struct Tools
{
	Tools();
	~Tools();
	/**
	* A helper method to calculate RMSE.
	*/
	Eigen::VectorXd CalculateRMSE		(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);
	/**
	* A helper method to calculate Jacobians.
	*/
	Eigen::MatrixXd CalculateJacobian	(Eigen::VectorXd& x_state);
	/**
	* A helper method to convert from Polar to Cartesian coordinate system.
	*/
	Eigen::VectorXd Polar2Cartesian_2D	(const Eigen::VectorXd& polar);
	/**
	* A helper method to convert from Cartesian to Polar coordinate system.
	*/
	void       		Cartesian2Polar_2D	(Eigen::VectorXd &polar,const Eigen::VectorXd& cartesian);
	
	Eigen::MatrixXd Hj;
	Eigen::VectorXd rmse;
};

#endif //__TOOLS_H__