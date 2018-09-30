#ifndef FusionEKF_H
#define FusionEKF_H

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
public:
  FusionEKF();
  virtual ~FusionEKF();

  // Sensor fusion 
  // Radar sensor measurements in the form of: (rho, phi, rho_dot)
  // Laser sensor measurements in the form of: (x, y)
  void ProcessMeasurement(const MeasurementPackage& measurement_pack);

  KalmanFilter ekf_;
private:
  bool is_initialized_;
  long long previous_timestamp_;
  Tools tools;

  // Measurement covariance matrix - laser (2x2)
  Eigen::MatrixXd R_laser_;

  // Measurement covariance matrix - radar (3x3)
  Eigen::MatrixXd R_radar_;

  // Measurement matrix - laser (2x4)
  Eigen::MatrixXd H_laser_;
};

#endif  // FusionEKF_H