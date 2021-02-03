#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() { Init(); }

UKF::~UKF() {}

void UKF::Init() {}

void UKF::GenerateSigmaPoints(MatrixXd *Xsig_out) {

  // set state dimension
  int n_x = 5;

  // define spreading parameter
  double lambda = 3 - n_x;

  // set example state
  VectorXd x = VectorXd(n_x);
  x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

  // set example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011,
      0.0071, 0.0060, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071,
      0.0007, 0.0098, 0.0100, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  // create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

  // calculate square root of P
  MatrixXd A = P.llt().matrixL();

  // calculate sigma points
  // set sigma points as columns of matrix Xsig
  Xsig.col(0) = x;
  for (int i = 0; i < n_x; i++) {
    Xsig.col(i + 1) = x + sqrt(lambda + n_x) * A.col(i);
  }
  for (int i = 0; i < n_x; i++) {
    Xsig.col(i + n_x + 1) = x - sqrt(lambda + n_x) * A.col(i);
  }

  // print result
  // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

  // write result
  *Xsig_out = Xsig;

  /**
   * expected result:
   * Xsig =
   *  5.7441  5.85768   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441
   * 5.7441   5.7441
   *    1.38  1.34566  1.52806     1.38     1.38     1.38  1.41434  1.23194     1.38
   * 1.38     1.38
   *  2.2049  2.28414  2.24557  2.29582   2.2049   2.2049  2.12566  2.16423  2.11398
   * 2.2049   2.2049 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015 0.55961
   * 0.371114 0.486077 0.407773   0.5015 0.3528 0.299973 0.462123 0.376339
   * 0.48417 0.418721 0.405627 0.243477 0.329261  0.22143 0.286879
   */
}

void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // Process noise covariance matrix
  MatrixXd Q = MatrixXd(2, 2);
  Q << std_a * std_a, 0, 0, std_yawdd * std_yawdd;
  // std::cout << "Q = " << std::endl << Q << std::endl;

  // set example state
  VectorXd x = VectorXd(n_x);
  x << 5.7441, 1.3800, 2.2049, 0.5015, 0.3528;

  // create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P << 0.0043, -0.0013, 0.0030, -0.0022, -0.0020, -0.0013, 0.0077, 0.0011,
      0.0071, 0.0060, 0.0030, 0.0011, 0.0054, 0.0007, 0.0008, -0.0022, 0.0071,
      0.0007, 0.0098, 0.0100, -0.0020, 0.0060, 0.0008, 0.0100, 0.0123;

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // create augmented mean state
  x_aug.head(n_x) = x;
  //   x_aug[n_x] = 0;
  //   x_aug[n_x + 1] = 0;

  // create augmented covariance matrix
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug.bottomRightCorner(2, 2) = Q;

  // std::cout << "P_aug = " << std::endl << P_aug << std::endl;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  // std::cout << "x_aug = " << std::endl << x_aug << std::endl;
  for (int i = 0; i < n_aug; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda + n_aug) * A.col(i);
    Xsig_aug.col(i + n_aug + 1) = x_aug - sqrt(lambda + n_aug) * A.col(i);
  }

  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;

  /**
   * expected result:
   *  Xsig_aug =
   * 5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052
   * 5.7441   5.7441   5.7441   5.7441   5.7441   5.7441 1.38  1.34566  1.52806
   * 1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38
   * 1.38     1.38     1.38 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049
   * 2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
   * 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015
   * 0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015 0.3528
   * 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627
   * 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528 0        0        0
   * 0        0        0  0.34641        0        0        0        0        0
   * 0 -0.34641        0 0        0        0        0        0        0        0
   * 0.34641        0        0        0        0        0        0 -0.34641
   */
}

void UKF::SigmaPointPrediction(MatrixXd *Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // create example sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);
  Xsig_aug << 5.7441, 5.85768, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441,
      5.63052, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 5.7441, 1.38, 1.34566,
      1.52806, 1.38, 1.38, 1.38, 1.38, 1.38, 1.41434, 1.23194, 1.38, 1.38, 1.38,
      1.38, 1.38, 2.2049, 2.28414, 2.24557, 2.29582, 2.2049, 2.2049, 2.2049,
      2.2049, 2.12566, 2.16423, 2.11398, 2.2049, 2.2049, 2.2049, 2.2049, 0.5015,
      0.44339, 0.631886, 0.516923, 0.595227, 0.5015, 0.5015, 0.5015, 0.55961,
      0.371114, 0.486077, 0.407773, 0.5015, 0.5015, 0.5015, 0.3528, 0.299973,
      0.462123, 0.376339, 0.48417, 0.418721, 0.3528, 0.3528, 0.405627, 0.243477,
      0.329261, 0.22143, 0.286879, 0.3528, 0.3528, 0, 0, 0, 0, 0, 0, 0.34641, 0,
      0, 0, 0, 0, 0, -0.34641, 0, 0, 0, 0, 0, 0, 0, 0, 0.34641, 0, 0, 0, 0, 0,
      0, -0.34641;

  // create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd::Zero(n_x, 2 * n_aug + 1);

  double delta_t = 0.1; // time diff in sec

  // predict sigma points
  VectorXd x(n_aug);
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    x = Xsig_aug.col(i);
    float v = x[2], psi = x[3], dpsi = x[4], nu_a = x[5], nu_dpsi = x[6];
    Xsig_pred.col(i) = x.head(n_x);
    if (fabs(dpsi) > 0.001) {
      Xsig_pred.col(i)[0] += v / dpsi * (sin(psi + dpsi * delta_t) - sin(psi));
      Xsig_pred.col(i)[1] += v / dpsi * (-cos(psi + dpsi * delta_t) + cos(psi));
      Xsig_pred.col(i)[3] += dpsi * delta_t;
    } else {
      Xsig_pred.col(i)[0] += v * cos(psi) * delta_t;
      Xsig_pred.col(i)[1] += v * sin(psi) * delta_t;
    }
    // add noise
    Xsig_pred.col(i)[0] += 1 / 2. * pow(delta_t, 2) * cos(psi) * nu_a;
    Xsig_pred.col(i)[1] += 1 / 2. * pow(delta_t, 2) * sin(psi) * nu_a;
    Xsig_pred.col(i)[2] += delta_t * nu_a;
    Xsig_pred.col(i)[3] += 1 / 2. * pow(delta_t, 2) * nu_dpsi;
    Xsig_pred.col(i)[4] += delta_t * nu_dpsi;
  }

  // print result
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  // write result
  *Xsig_out = Xsig_pred;

  //   expected result:
  // Xsig_pred =

  // 5.93553 6.06251 5.92217 5.9415 5.92361 5.93516 5.93705 5.93553 5.80832 5.94481
  // 5.92935 5.94553 5.93589 5.93401 5.93553

  // 1.48939 1.44673 1.66484 1.49719 1.508 1.49001 1.49022 1.48939 1.5308 1.31287
  // 1.48182 1.46967 1.48876 1.48855 1.48939

  // 2.2049 2.28414 2.24557 2.29582 2.2049 2.2049 2.23954 2.2049 2.12566 2.16423 2.11398
  // 2.2049 2.2049 2.17026 2.2049

  // 0.53678 0.473387 0.678098 0.554557 0.643644 0.543372 0.53678 0.538512
  // 0.600173 0.395462 0.519003 0.429916 0.530188 0.53678 0.535048

  // 0.3528 0.299973 0.462123 0.376339 0.48417 0.418721 0.3528 0.387441 0.405627
  // 0.243477 0.329261 0.22143 0.286879 0.3528 0.318159
}