#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "kalman.cpp"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

int main() {
  vector<VectorXd> measurements;
  VectorXd single_meas(1);
  single_meas << 1;
  measurements.push_back(single_meas);
  single_meas << 2;
  measurements.push_back(single_meas);
  single_meas << 3;
  measurements.push_back(single_meas);
  Kalman obj = Kalman();
  // call Kalman filter algorithm
  obj.filter(measurements);
  return 0;
}

