#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_x_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long previous_timestamp_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  static constexpr int n_x_ = 5;

  ///* number of new states
  static constexpr int n_aug_delta_ = 2 ;
  ///* Augmented state dimension
  static constexpr int n_aug_ = n_x_ + n_aug_delta_;

  ///* Sigma point spreading parameter
  static constexpr double lambda_= 3 - n_x_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  ///* acceleration noise for prediction model
  const MatrixXd Q_;
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  /**
   * augment sigma points with noise 
   * @param Xsig_out the augmented sigma point output
   */
  void GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) ;


  /**
   * Predict states based on the augmented sigma points 
   * @param Xsig_aug the augmented sigma point 
   * @param delta_t  the elapsed time in seconds since the last measurement 
   * @param Xsig_out the predicted sigma states 
   */
  void PredictAugmentedSigmaPoints(const MatrixXd& Xsig_aug, double delta_t,  MatrixXd* Xsig_out); 


  /**
   * Calculate state means and state certainty covariance based on the predicted augmented sigma points   
   * @param x_sig_pred  the predicted sigma points
   * @param x_out       the system state means
   * @param P_out       the prediction uncertainty covariance

   */
  void CalculatePredictionMeanAndCovariance(const MatrixXd& x_sig_pred, VectorXd* x_out, MatrixXd* P_out);

};

#endif /* UKF_H */
