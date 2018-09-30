#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;

  R_laser_ = MatrixXd(2,2);
  R_radar_ = MatrixXd(3,3);
  H_laser_ = MatrixXd(2,4);
  ekf_.P_ = MatrixXd(4,4);
  ekf_.F_ = MatrixXd(4,4);

  R_laser_ << 0.0225, 0,
              0, 0.0225;

  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage& measurement_pack) {
  if (!is_initialized_) {
      std::cout << "EKF: \n";
      ekf_.x_ = VectorXd(4);

      if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          // Polar to cartesian coordinates
          float rho = measurement_pack.raw_measurements_(0);
          float phi = measurement_pack.raw_measurements_(1);
          float rho_dot = measurement_pack.raw_measurements_(2);
          ekf_.x_ << rho*cos(phi), rho*sin(phi), 0, 0;
      }
      else {
          ekf_.x_ << measurement_pack.raw_measurements_(0), 
                     measurement_pack.raw_measurements_(1), 0, 0;
      }

      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
      return;
  }

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  int noise_ax = 9;
  int noise_ay = 9;
  float dt_2 = dt*dt;
  float dt_3 = dt*dt_2;
  float dt_4 = dt*dt_3;

  // Updates state transition matrix
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // Sets process covariance matrix
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
             0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
             dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 
  else {
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
