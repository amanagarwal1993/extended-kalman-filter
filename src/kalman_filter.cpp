#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    P_ = F_ * P_ * F_.transpose() + Q_;
    return;
}

// For Laser measurements
void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using regular Kalman Filter equations
  */
    // Takes in Z, and updates X, P
    VectorXd y;
    y = z - x_;
    MatrixXd S;
    S = H_*P_*H_.transpose() + R_;
    MatrixXd K;
    K = P_*H_.transpose()*S.inverse();
    
    x_ = x_ + (K * y);
    P_ = (P_*P_.setIdentity() - K*H_) * P_;
    return;
}

// For Radar measurements
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using EXTENDED Kalman Filter equations
   // might require Jacobian Matrix?
  */
    // Takes in Z, and updates X, P
    MatrixXd Hj(3,4);
    Hj = Tools().CalculateJacobian(x_);
    
    VectorXd y;
    y = z - Hj*x_;
    if (y[1] > 180) {
        y[1] = 360 - y[1];
    }
    
    MatrixXd S;
    S = Hj*P_*Hj.transpose() + R_;
    
    MatrixXd K;
    K = P_*Hj.transpose()*S.inverse();
    
    x_ = x_ + (K * y);
    x_ = Hj.inverse() * x_;
    
    P_ = (P_*P_.setIdentity() - K*H_) * P_;
    P_ = Hj.inverse() * P_;
    return;
    
}
