#include "kalman_filter.h"


KalmanFilter::KalmanFilter(){}
KalmanFilter::~KalmanFilter(){}

void KalmanFilter::init(const Eigen::VectorXd &x_in, const Eigen::MatrixXd &P_in, const Eigen::MatrixXd &F_in, const Eigen::MatrixXd &Q_in)
{
	x = x_in;
	P = P_in;
	F = F_in;
	Q = Q_in;

	x << 1, 1, 1, 1;

	P << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1000, 0,
		 0, 0, 0, 1000;

	F << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 0, 0, 0, 1;

	Q << 1, 0, 1, 0,
		 0, 1, 0, 1,
		 1, 0, 1, 0,
		 0, 1, 0, 1;
}
void KalmanFilter::predict()
{
	x = F*x;
	P = F*P*F.transpose() + Q;
}
void KalmanFilter::updateKF(const Eigen::VectorXd &z)
{
	update(z - (H * x));
}
void KalmanFilter::updateEKF(const Eigen::VectorXd &z) 
{
	Eigen::VectorXd polar(3);
	
	tools.Cartesian2Polar_2D(polar,x);
	
	polar = z - polar;
	
	if	   (polar(1) > pi) polar(1)  -= 2*pi;
	else if(polar(1) < -pi) polar(1) += 2*pi;
	
	update(polar);
}
void KalmanFilter::update(const Eigen::VectorXd &y)
{
	Eigen::MatrixXd Ht = H.transpose();
	Eigen::MatrixXd S  = H* P * Ht + R;
	Eigen::MatrixXd K  = P * Ht * S.inverse();
	Eigen::MatrixXd I  = Eigen::MatrixXd::Identity(x.size(), x.size());

	x = x + (K * y);
	P = (I - K * H) * P;
}