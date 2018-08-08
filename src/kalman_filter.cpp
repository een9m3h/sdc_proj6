#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &Hj_in, MatrixXd &H_laser_in,
						MatrixXd &R_radar_in, MatrixXd &R_laser_in,
						MatrixXd &Q_in) {
  x_ 		= x_in;
  P_ 		= P_in;
  F_ 		= F_in;
  Hj_ 		= Hj_in;
  H_laser_ 	= H_laser_in;
  R_radar_  = R_radar_in;
  R_laser_  = R_laser_in;
  Q_ 		= Q_in;
}

void KalmanFilter::Predict() {
	// KF Prediction step
	x_ = F_*x_;
	P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

	MatrixXd H_ = H_laser_;
	MatrixXd R_ = R_laser_;
	VectorXd y_;
	MatrixXd K_,S_;
	MatrixXd I = MatrixXd(4,4);
	I << 1,0,0,0,
	     0,1,0,0,
		 0,0,1,0,
		 0,0,0,1;

	// KF Measurement update step
	y_ = z - H_*x_;
	S_ = H_*P_*H_.transpose() + R_;
	K_ = P_*H_.transpose()*S_.inverse();
	x_ = x_ + K_*y_;
	P_ = (I-K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  std::cout << "TODO:update the state by using Extended Kalman Filter equations" << std::endl;
}
