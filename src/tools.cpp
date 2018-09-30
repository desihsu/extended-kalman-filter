#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const std::vector<VectorXd>& estimations,
                              const std::vector<VectorXd>& ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  for (int i = 0; i < estimations.size(); ++i) {
      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array() * residual.array();
      rmse += residual;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float a = px*px + py*py;
  float b = sqrt(a);
  float c = b*a;
  
  Hj << px/b, py/b, 0, 0,
        -py/a, px/a, 0, 0,
        py*(vx*py-vy*px)/c, px*(vy*px-vx*py)/c, px/b, py/b;

  return Hj;
}
