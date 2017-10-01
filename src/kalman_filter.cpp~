#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
#define PI 3.1415926

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd F_t  =  F_.transpose();
  P_ = F_* P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_*P_*H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_ * H_t * S_i;
  
  //new estimations
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size,x_size);
  P_ = (I - K*H_)*P_;  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_pred(3);  // the h(x)
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  
  float phi = 0.0;
  if(fabs(x_(0))< 0.001){
      phi = 0; 
  }
  else{
  phi = atan2(x_(1),x_(0));
  }// the output stays between [-pi,pi]
  
  float rhodot = 0.0;
  if (fabs(rho)< 0.0001){
      rhodot = 0;
  }
  else{  
      rhodot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  
  z_pred<<rho, phi, rhodot;
  
  VectorXd y = z - z_pred;
  //normalize the vector y between (-PI,PI) just as phi in z_pred[1] did
 /* if (y(1) < - 1*PI){
      y(1)= y(1)+2*PI;
  }
  if (y(1)> PI){
      y(1) = y(1) - 2*PI;
  }*/
  y(1) = atan2(sin(y(1)),cos(y(1)));
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_*P_*H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_*H_t*S_i;
  
  //new estimations
  x_ = x_+ (K*y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_)*P_;    
}
