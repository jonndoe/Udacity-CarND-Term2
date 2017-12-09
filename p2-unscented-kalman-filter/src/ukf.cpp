#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // ** Needs tuning **
  std_a_ = 2.;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // ** Needs tuning **
  std_yawdd_ = 0.6;

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // time when the state is true, in us
  time_us_ = 0.0;

  // state dimension
  n_x = 5;

  // augmented dimension
  n_aug = 7;

  // spreading parameter
  lambda = 3 - n_x_;

  // sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
      double weight = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight;
  }

  // measurement noise matrices
  R_laser = MatrixXd(2, 2);
  R_laser << std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;

  R_radar = MatrixXd(3, 3);
  R_radar << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;

  // process noise covariance matrix
  Q = MatrixXd(2, 2);
  Q << std_a_ * std_a_, 0,
          0, std_yawdd_ * std_yawdd_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  x_ = VectorXd(n_x_);

  // state covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;

  // NIS
  NIS_radar_ = 0.0;
  NIS_laser_ = 0.0;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MeasurementUpdate(meas_package);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MeasurementUpdate(meas_package);

}

// single update measurement function for both lidar and radar
void UKF::MeasurementUpdate(MeasurementPackage meas_package) {
  VectorXd z;
  MatrixXd z_sig;
  VectorXd z_pred;
  MatrixXd R;
  int n_z;

  if (meas_package.sensor_type_ == meas_package.LASER) {
      n_z = 3;
      R = R_laser;

      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];

      z = VectorXd(n_z);
      z << px, py;

      z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
      z_pred = VectorXd(n_z);

      for (int i = 0; i < 2 * n_aug_ + 1; i++) {
          // measurement model
          z_sig(0, i) = Xsig_pred_(0, i);
          z_sig(1, i) = Xsig_pred_(1, i);
      }
    } else if (meas_package.sensor_type_ == meas_package.RADAR) {
          n_z = 2;
          R = R_radar;

          // Read Measurements
          double rho = meas_package.raw_measurements_[0];
          double phi = meas_package.raw_measurements_[1];
          double v = meas_package.raw_measurements_[2];

          z = VectorXd(n_z);
          z << rho, phi, v;

          // predict radar measurement
          z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
          z_pred = VectorXd(n_z);

          for (int i = 0; i < 2 * n_aug_ + 1; i++) {
              double p_x = Xsig_pred_(0, i);
              double p_y = Xsig_pred_(1, i);
              double vel = Xsig_pred_(2, i);
              double yaw = Xsig_pred_(3, i);
              double v_x = cos(yaw) * vel;
              double v_y = sin(yaw) * vel;

              // rho
              z_sig(0, i) = sqrt(p_x * p_x + p_y * p_y);

              // phi
              z_sig(1, i) = atan2(p_y, p_x);

              // rho_dot
              z_sig(2, i) = (z_sig(0, i) < 0.0001 ? (p_x * v_x + p_y * v_y) / 0.0001 : \
              (p_x * v_x + p_y * v_y) / z_sig(0, i));
          }
    }

    //mean predicted measurement
    z_pred.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred = z_pred + weights_(i) * z_sig.col(i);
    }

    //measurement covariance matrix
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    // Cross correlation matrix Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = z_sig.col(i) - z_pred;
        S = S + weights_(i) * z_diff * z_diff.transpose();

        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI); // normalize angle
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R;

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // normalize angle
    if (meas_package.sensor_type_ == meas_package.RADAR) {
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);
    }

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    // Calculating NIS
    string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    VectorXd nis = z_diff.transpose() * S.inverse() * z_diff;
    cout << sensor_type << " measurement : " << nis << endl;
}
