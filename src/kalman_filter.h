#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "Eigen/Dense"

class KalmanFilter {
public:
  KalmanFilter();
  virtual ~KalmanFilter();

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transition matrix
  Eigen::MatrixXd F_;

  // Process covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix
  Eigen::MatrixXd H_;

  // Measurement covariance matrix
  Eigen::MatrixXd R_;

  void Predict();

  // Update step for laser sensor
  void Update(const Eigen::VectorXd& z);

  // Update step for radar sensor 
  // Uses a Jacobian matrix for nonlinear state estimation
  void UpdateEKF(const Eigen::VectorXd& z);
};

#endif  // KALMAN_FILTER_H
