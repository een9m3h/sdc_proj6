#include "kalman_filter.h"
#include <iostream>
#include <math.h>
#include "tools.h"

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
  I_ 		= MatrixXd(4,4);
  I_ << 1,0,0,0,
	     0,1,0,0,
		 0,0,1,0,
		 0,0,0,1;
  
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
	
	
	
	// KF Measurement update step
	y_ = z - H_*x_;
	S_ = H_*P_*H_.transpose() + R_;
	K_ = P_*H_.transpose()*S_.inverse();
	x_ = x_ + K_*y_;
	P_ = (I_-K_*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
	Tools tools;
	MatrixXd H_ = tools.CalculateJacobian(x_);
	MatrixXd R_ = R_radar_;
	VectorXd y_, x_polar_ = VectorXd(3);
	MatrixXd K_,S_;
	
	//recover state parameters
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = px*vx+py*vy;
	
	x_polar_(0) = c2;
	x_polar_(1) = atan2(py,px);
	x_polar_(2) = c3/c2;
	
	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "Calculate EKF measurement - Error - Division by Zero" << endl;
		return;
	}
  
	// EKF Measurement update step
	y_ = z - x_polar_;
	y_(1) = RangeAngle((float)y_(1));
	S_ = H_*P_*H_.transpose() + R_;
	K_ = P_*H_.transpose()*S_.inverse();
	x_ = x_ + K_*y_;
	P_ = (I_-K_*H_)*P_;
  
  std::cout << "TODO:update the state by using Extended Kalman Filter equations" << std::endl;
}

float KalmanFilter::RangeAngle(float phi){
	const float PI_F=3.14159265358979f;
	while(phi < -PI_F || phi > PI_F){
		if(phi > PI_F) 
			phi -= PI_F;
		else
			phi += PI_F;
		
	}
	return phi;
}
