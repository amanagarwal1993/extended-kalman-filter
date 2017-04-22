#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
    
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    //cout<<"R matrices initialized \n";
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    if (!is_initialized_) {
        //KalmanFilter ekf_; // To initialize a kalman filter, along with all its variable names.

        // first measurement
        //cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        //cout<<ekf_.x_<<" X "<<endl;

        ekf_.F_ = MatrixXd(4,4);
        ekf_.F_<< 1.,0,1.,0,
                0,1.,0,1.,
                0,0,1.,0,
                0,0,0,1.;
        //cout<<ekf_.F_<<" F \n"<<endl;

        ekf_.H_ = MatrixXd(2,4);
        ekf_.H_ << 1,0,0,0,
                0,1,0,0;
        //cout<<ekf_.H_<<" H \n"<<endl;

        ekf_.P_ = MatrixXd(4,4);
        ekf_.P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 100, 0,
                0, 0, 0, 100;
        //cout<<ekf_.P_<<" P \n"<<endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
        Convert radar from polar to cartesian coordinates and initialize x_ with first measurement.
      */
        //cout<<"Radar \n";

        VectorXd x(3);
        x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];

        //MatrixXd Hj;
        //Tools tools;
        //Hj = tools.CalculateJacobian(ekf_.x_);
        //cout << Hj << " Hj \n"<<endl;

        ekf_.x_ << x[0]*cos(x[1]), x[0]*sin(x[1]), 0, 0;

        //cout<<ekf_.x_<<" After adding radar values to it \n"<<endl;
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize with first measurement.
      */
        VectorXd x(4);
        
        x << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        if (x[0] < 0.0001) {
            x[0] = 0.0001;
        }
        if (x[1] < 0.0001) {
            x[1] = 0.0001;
        }
        ekf_.x_ << x[0], x[1], 0, 0;
        //cout<<ekf_.x_<<" After adding laser values to it \n"<<endl;
    }
      // Setting the first timestamp
      previous_timestamp_ = measurement_pack.timestamp_;
      //cout<<"\n"<<previous_timestamp_<<" Timestamp"<<endl;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    //cout<<"\n Now we begin the iterations: \n\n";
    return;
  }

    else {
        // What to do if filter has already been initialized
        //cout<<"Iteration after initialization"<<endl;

        // First, calculate change in time
        //cout<<"*****New Iteration*****\n";
        double dtime;

        long newtime = measurement_pack.timestamp_; // New timestamp

        dtime = (newtime - previous_timestamp_) / 1000000.0;
        
        // If time gap is too small, skip predict step
        if (dtime > 0) {
        
        double dtime2 = dtime * dtime;
        double dtime3 = dtime2 * dtime / 2;
        double dtime4 = dtime2 * dtime2 / 4;

        int noise_ax, noise_ay;
        noise_ax = 9.;
        noise_ay = 9.;


        ekf_.Q_ = MatrixXd(4,4);
        ekf_.Q_ << (dtime4 * noise_ax), 0., (dtime3 * noise_ax), 0,
                    0, (dtime4 * noise_ay), 0, (dtime3 * noise_ay),
                    (dtime3 * noise_ax), 0, (dtime2 * noise_ax), 0,
                    0, (dtime3 * noise_ay), 0, (dtime2 * noise_ay);
        //cout<<ekf_.Q_<<"  Q \n";

        ekf_.F_(0,2) = dtime;
        ekf_.F_(1,3) = dtime;


        ekf_.Predict();
        //cout<<ekf_.x_<<" Predicted x\n"; **********************************************************
        };
        
        Hj_ = MatrixXd(3, 4);



      if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
          //cout<<"Radar\n";
          VectorXd z(3);
          z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
          //cout<<z<<"Radar measurements\n";
          ekf_.R_ = R_radar_;
          ekf_.UpdateEKF(z);
      } else {
        // Laser updates
          //cout<<"Laser\n";
          VectorXd z(2);
          z << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
          ekf_.R_ = R_laser_;
          ekf_.Update(z);
      }

        previous_timestamp_ = newtime;

      // print the output
      //cout << "x_ = " << ekf_.x_ <<"\n"<<endl; **********************************************************
      //cout << "P_ = " << ekf_.P_ <<"\n"<< endl;
      return;
    }
}
