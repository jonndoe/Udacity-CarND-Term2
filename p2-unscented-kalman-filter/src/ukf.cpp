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

  // process noise standard deviation longitudinal acceleration in m/s^2 (!! needs tuning !!)
  std_a_ = 2.;

  // process noise standard deviation yaw acceleration in rad/s^2 (!! needs tuning !!)
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
  n_x_ = 5;

  // augmented dimension
  n_aug_ = 7;

  // spreading parameter
  lambda_ = 3 - n_x_;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
      double weight = 0.5 / (n_aug_ + lambda_);
      weights_(i) = weight;
  }

  // measurement noise matrices
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
          0, std_laspy_ * std_laspy_;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
          0, std_radphi_ * std_radphi_, 0,
          0, 0, std_radrd_ * std_radrd_;

  // process noise covariance matrix
  Q_ = MatrixXd(2, 2);
  Q_ << std_a_ * std_a_, 0,
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

  if (!is_initialized_) {
      // switch between lidar and radar
      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
          double px = meas_package.raw_measurements_[0];
          double py = meas_package.raw_measurements_[1];
          x_ << px, py, 0, 0, 0;
      } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
          double rho = meas_package.raw_measurements_[0];
          double phi = meas_package.raw_measurements_[1];
          double v = meas_package.raw_measurements_[2];
          double px = rho * cos(phi);
          double py = rho * sin(phi);
          double yawd = 0.0;
          x_ << px, py, v, phi, yawd;
      }
      previous_timestamp_ = meas_package.timestamp_;
      is_initialized_ = true;
      return;
  }

  if ((use_radar_ && meas_package.sensor_type_ == meas_package.RADAR) ||
      (use_laser_ && meas_package.sensor_type_ == meas_package.LASER)) {
      double delta_t = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
      previous_timestamp_ = meas_package.timestamp_;

      Prediction(delta_t);
      MeasurementUpdate(meas_package);
  }
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

  /*****  Generate Sigma Points  *****/

  //create sigma point matrix
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);

  //calculate square root of P
  MatrixXd A = P_.llt().matrixL();

  //set lambda for non-augmented sigma points
  lambda_ = 3 - n_x_;

  //set first column of sigma point matrix
  Xsig.col(0) = x_;

  //set remaining sigma points
  for (int i = 0; i < n_x_; i++)
  {
    Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }

  /***** Augment Sigma Points *****/

  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //set lambda for augmented sigma points
  lambda_ = 3 - n_aug_;

  //create augmented mean state
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5, 5) = P_;
  P_aug(5, 5) = std_a_*std_a_;
  P_aug(6, 6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  /***** Predict Sigma Points *****/

  //predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug(0, i);
    double p_y      = Xsig_aug(1, i);
    double v        = Xsig_aug(2, i);
    double yaw      = Xsig_aug(3, i);
    double yawd     = Xsig_aug(4, i);
    double nu_a     = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  }

  /***** Convert Predicted Sigma Points to Mean/Covariance *****/

  //set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {  //2n+1 weights
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    //state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
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

//single MeasurementUpdate function for both lidar and radar (to reduce duplicate code)
void UKF::MeasurementUpdate(MeasurementPackage meas_package) {
  VectorXd z;
  MatrixXd z_sig;
  VectorXd z_pred;
  MatrixXd R;
  int n_z;

  if (meas_package.sensor_type_ == meas_package.LASER) {
      n_z = n_z_lidar_;
      R = R_laser_;

      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];

      z = VectorXd(n_z);
      z << px, py;

      z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
      z_pred = VectorXd(n_z);

      for (int i = 0; i < 2 * n_aug_ + 1; i++) {
          //measurement model
          z_sig(0, i) = Xsig_pred_(0, i);
          z_sig(1, i) = Xsig_pred_(1, i);
      }
    } else if (meas_package.sensor_type_ == meas_package.RADAR) {
          n_z = n_z_radar_;
          R = R_radar_;

          //read measurements
          double rho = meas_package.raw_measurements_[0];
          double phi = meas_package.raw_measurements_[1];
          double v = meas_package.raw_measurements_[2];

          z = VectorXd(n_z);
          z << rho, phi, v;

          //predict radar measurement
          z_sig = MatrixXd(n_z, 2 * n_aug_ + 1);
          z_pred = VectorXd(n_z);

          for (int i = 0; i < 2 * n_aug_ + 1; i++) {
              double p_x = Xsig_pred_(0, i);
              double p_y = Xsig_pred_(1, i);
              double vel = Xsig_pred_(2, i);
              double yaw = Xsig_pred_(3, i);
              double v_x = cos(yaw) * vel;
              double v_y = sin(yaw) * vel;

              //rho
              z_sig(0, i) = sqrt(p_x * p_x + p_y * p_y);

              //phi
              z_sig(1, i) = atan2(p_y, p_x);

              //rho_dot
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

    //cross correlation matrix Tc
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

    //Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    //residual
    VectorXd z_diff = z - z_pred;

    //angle normalization
    if (meas_package.sensor_type_ == meas_package.RADAR) {
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);
    }

    //calculate NIS
    string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    VectorXd nis = z_diff.transpose() * S.inverse() * z_diff;
    cout << sensor_type << " measurement : " << nis << endl;

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

}
