#include "tools.h"

Tools::Tools() : Hj(Eigen::MatrixXd::Zero(3, 4)),rmse(Eigen::VectorXd::Zero(4)){}
Tools::~Tools(){}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const  std::vector<Eigen::VectorXd> &ground_truth) 
{
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	for (unsigned int i = 0; i < estimations.size(); ++i)
		rmse.array() += (estimations[i] - ground_truth[i]).array().pow(2.0);

	rmse.array() = (rmse / estimations.size()).array().sqrt();

	return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(Eigen::VectorXd& x_state) 
{
	const double c_1 = std::pow(x_state(0), 2.0) + std::pow(x_state(1), 2.0);
	const double c_2 = std::sqrt(c_1);
	const double c_3 = c_1*c_2;
	
	if (std::fabs(c_1) < 0.0001) return Hj;

	Hj(0, 0) = x_state(0) / c_2;
	Hj(0, 1) = x_state(1) / c_2;
	Hj(1, 0) = -(x_state(1) / c_1);
	Hj(1, 1) = x_state(0) / c_1;
	Hj(2, 0) = x_state(1)*(x_state(2)*x_state(1) - x_state(3)*x_state(0)) / c_3;
	Hj(2, 1) = x_state(0)*(x_state(0)*x_state(3) - x_state(1)*x_state(2)) / c_3;
	Hj(2, 2) = Hj(0, 0);
	Hj(2, 3) = Hj(0, 1);

	return Hj;
}

Eigen::VectorXd Tools::Polar2Cartesian_2D(const Eigen::VectorXd& polar)
{
	Eigen::VectorXd cartesian(4);
	cartesian<< polar(0) * std::cos(polar(1)), polar(0) * std::sin(polar(1)), polar(2) * std::cos(polar(1)), polar(2) * std::sin(polar(1));
	return cartesian;
}

void Tools::Cartesian2Polar_2D(Eigen::VectorXd &polar,const Eigen::VectorXd &cartesian)
{
	const double rho		= std::sqrt(std::pow(cartesian(0), 2.0) + std::pow(cartesian(1), 2.0));
	const double theta		= std::atan2(cartesian(1),cartesian(0));
	const double rho_dot	= (cartesian(0)*cartesian(2) + cartesian(1)*cartesian(3)) / rho;

	polar << rho,theta,rho_dot;
}