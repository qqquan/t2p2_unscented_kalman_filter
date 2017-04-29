#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

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

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_x_initialized_ = false; 

  // n_x_ = 5; // CVTR model has five states: px, py, v, yaw, yawd
  // n_aug_delta_ = 2; //acceleration noise has two dimensions
  // n_aug_ = n_x_ + n_aug_delta_; 
   

  P_.resize(n_x_, n_x_); //we are only certain about px and py at beginning. a large value, e.g. 1000, means high uncertainty.
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1000, 0, 0,
        0, 0, 0, 1000, 0,
        0, 0, 0, 0, 1000;



     
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

  if (!is_x_initialized_)
  {
    /**
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    double init_x = 0.0;
    double init_y = 0.0;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];

      init_x = rho*std::cos(phi);
      init_y = rho*std::sin(phi);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      init_x = meas_package.raw_measurements_[0];
      init_y = meas_package.raw_measurements_[1];
    }

    x_ << init_x, init_y, 0 , 0, 0;
    previous_timestamp_us_ = meas_package.timestamp_;
    
    return; 
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
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  PredictAugmentedSigmaPoints(Xsig_aug, delta_t, &Xsig_pred_);
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
}





void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) {

 
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create augmented mean state
  x_aug << x_, 0, 0;
  //create augmented covariance matrix
  MatrixXd Q(n_aug_delta_,n_aug_delta_);
  Q << std_a_*std_a_, 0,
       0,             std_yawdd_*std_yawdd_;
       
  P_aug.block<n_x_,n_x_>(0,0) = P_;    
  P_aug.block<n_aug_delta_,n_aug_delta_>(n_x_,n_x_) = Q;    
    
  //create square root matrix
  MatrixXd P_aug_sqrt = P_aug.llt().matrixL();
  MatrixXd P_delta = std::sqrt(lambda_+n_aug_)*P_aug_sqrt;    
  //create augmented sigma points
  
  Xsig_out->col(0) = x_aug;
  
  for(int i=0; i<n_aug_; i++)
  {
    Xsig_out->col(1+i) =   x_aug + P_delta.col(i);
    Xsig_out->col(1+i+n_aug_) = x_aug - P_delta.col(i);
    
  }

  // //write result
  // *Xsig_out = Xsig_aug;


}


void UKF::PredictAugmentedSigmaPoints(const MatrixXd& Xsig_aug, double delta_t, MatrixXd* Xsig_out) 
{

  //create a temporary matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);


  //predict sigma points
  //avoid division by zero
  //write predicted sigma points into right column
  for(int i = 0; i < (2*n_aug_ + 1); i++ )
  {
      double px       = Xsig_aug.col(i)[0];
      double py       = Xsig_aug.col(i)[1];
      double v        = Xsig_aug.col(i)[2];
      double yaw      = Xsig_aug.col(i)[3];
      double yaw_rate = Xsig_aug.col(i)[4];
      double niu_a    = Xsig_aug.col(i)[5];
      double niu_yaw  = Xsig_aug.col(i)[6];    

      double px_new       = 0.0;
      double py_new       = 0.0;
      double v_new        = 0.0;
      double yaw_new      = 0.0;
      double yaw_rate_new = 0.0;
      double niu_a_new    = 0.0;
      double niu_yaw_new  = 0.0;  
      
      if (yaw_rate != 0)
      {
         px_new       = px + v/yaw_rate*( sin(yaw + yaw_rate*delta_t) - sin(yaw)) + 0.5 *delta_t*delta_t*cos(yaw)*niu_a;
         py_new       = py + v/yaw_rate*(-cos(yaw + yaw_rate*delta_t) + cos(yaw)) + 0.5 *delta_t*delta_t*sin(yaw)*niu_a;
         v_new        = v + delta_t*niu_a;
         yaw_new      = yaw + yaw_rate*delta_t + 0.5*delta_t*delta_t*niu_yaw;
         yaw_rate_new = yaw_rate + delta_t*niu_yaw;
         
      }
      else
      {
         px_new       = px + v*cos(yaw)*delta_t + 0.5*delta_t*delta_t*cos(yaw)*niu_a; 
         py_new       = px + v*sin(yaw)*delta_t + 0.5*delta_t*delta_t*sin(yaw)*niu_a; 
         v_new        = v + delta_t*niu_a;
         yaw_new      = yaw + yaw_rate*delta_t + 0.5*delta_t*delta_t*niu_yaw;
         yaw_rate_new = yaw_rate + delta_t*niu_yaw;
      }
      
      Xsig_pred.col(i)<<px_new, py_new, v_new, yaw_new, yaw_rate_new;
      
  }

  //write result
  *Xsig_out = Xsig_pred; //copy assignment. TODO: Make a move assignment

}