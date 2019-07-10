#include <iostream>
#include <vector>
#include "Eigen/Dense"

using std::cout;
using std::endl;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Kalman{

public:
  // Kalman Filter variables
  VectorXd x;	// object state
  MatrixXd P;	// object covariance matrix
  VectorXd u;	// external motion
  MatrixXd F; // state transition matrix
  MatrixXd H;	// measurement matrix
  MatrixXd R;	// measurement covariance matrix
  MatrixXd I; // Identity matrix
  MatrixXd Q;	// process covariance matrix
  
  // constructor, hard coded to work on an 1D Kalman filter
  Kalman(){
    //position
    x = VectorXd(2);
    x << 0, 0;
    //co-variance
    P = MatrixXd(2, 2);
    P << 1000, 0, 0, 1000;
    //external motion
    u = VectorXd(2);
    u << 0, 0;
    //state transition matrix
    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;
    //measurement matrix
    H = MatrixXd(1, 2);
    H << 1, 0;
    //measurement co variance
    R = MatrixXd(1, 1);
    R << 1;
    //identity matrix
    I = MatrixXd::Identity(2, 2);
    //process co-variance matrix
    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;

  }

  void filter(vector <VectorXd> &measurements) {
    for (unsigned int n = 0; n < measurements.size(); ++n) {
      VectorXd z = measurements[n];
      measure(z);
      predict();
      cout << "x=" << endl <<  x << endl;
      cout << "P=" << endl <<  P << endl;
    }
  }

  void predict(){
    x = F * x + u;
    P = F * P * F.transpose() + Q;
  }
  void measure(const VectorXd z){
    VectorXd y = z - H * x;
    MatrixXd S = H*P*H.transpose() + R;
    MatrixXd K = P*H.transpose()*S.inverse();

    x = x + (K * y);
    P = (I - K*H) * P;

    return;
  }
};
