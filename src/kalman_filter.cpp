#include "kalman_filter.h"
#include "tools.h"
#include "iostream"

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
    y = z - H_*x_;
    MatrixXd S;
    S = H_*P_*H_.transpose() + R_;
    MatrixXd K;
    K = P_*H_.transpose()*S.inverse();
    
    MatrixXd I(4,4);
    I << 1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;

    
    x_ = x_ + (K * y);
    P_ = (I - K*H_) * P_;
    //std::cout<<P_<<"-- P \n";
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
    
    VectorXd zp(3);
    double px = x_[0];
    double py = x_[1];
    double vx = x_[2];
    double vy = x_[3];
    
    double z1 = sqrt((px*px) + (py*py));
    double z2 = atan2(py, px);
    double z3 = (px*vx + py*vy)/z1;
    
    zp << z1, z2, z3;
    
    VectorXd y;
    y = z - zp;
    std::cout<<"****\n"<<y[1]<<"Z1 \n";
    if (y[1] < -M_PI/2) {
        y[1] = y[1] + 2*M_PI;
    }
    if (y[1] > M_PI/2) {
        y[1] = y[1] - 2*M_PI;
    }
    std::cout<<y[1]<<"Z2 \n";
    
    MatrixXd S;
    S = Hj*P_*Hj.transpose() + R_;
    
    MatrixXd K;
    K = P_*Hj.transpose()*S.inverse();
    
    x_ = x_ + (K * y);
    //x_ = Hj.inverse() * x_;
    
    MatrixXd I(4,4);
    I << 1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1;

    
    P_ = (I - K*Hj) * P_;
    //std::cout<<P_<<"-- P\n";
    //P_ = Hj.inverse() * P_;
    return;
    
}
