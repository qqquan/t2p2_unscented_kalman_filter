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
  std_a_ = 0.3; // cause huge value in Xsig

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03; //  assignment value: 0.0175;


  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3; // assignment value: 0.1



  is_x_initialized_ = false; 


  P_.resize(n_x_, n_x_); //we are only certain about px and py at beginning. a large value, e.g. 1000, means high uncertainty.
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  Q_.resize(n_aug_delta_,n_aug_delta_);
  Q_ << std_a_*std_a_, 0,
        0,             std_yawdd_*std_yawdd_;
       

  weights_ = VectorXd(2*n_aug_+1);
  
  R_radar_.resize(n_z_radar_, n_z_radar_);
  R_radar_ << std_radr_*std_radr_,  0,                        0,
              0,                    std_radphi_*std_radphi_,  0,
              0,                    0,                        std_radrd_*std_radrd_;

  R_lidar_.resize(n_z_lidar_, n_z_lidar_);
  R_lidar_ << std_laspx_*std_laspx_,    0,
              0,                        std_laspy_*std_laspy_;

  H_lidar_.resize(n_z_lidar_, n_x_);
  H_lidar_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0;     
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {


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

    if( (init_x < 0.001) && (init_y < 0.001) )
    {
      //invalid data, discard
      is_x_initialized_ = false;
    }
    else
    {
      x_ << init_x, init_y, 0 , 0, 0;
      previous_timestamp_us_ = meas_package.timestamp_;

      // done initializing, no need to predict or update
      is_x_initialized_ = true;
    }

  }
  else
  {
    double delta_t_sec = (meas_package.timestamp_ - previous_timestamp_us_) / 1000000.0;
    previous_timestamp_us_ = meas_package.timestamp_;

    //numerical stability fix from Wolfgang_Steiner. reference: https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/3
    // while (delta_t_sec > 0.1)
    // {
    // const double dt = 0.05;
    // Prediction(dt);
    // delta_t_sec -= dt;
    // }
    Prediction(delta_t_sec);
 
    if (meas_package.sensor_type_ ==  MeasurementPackage::RADAR)
    {
      UpdateRadar(meas_package);

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      UpdateLidar(meas_package);
    }
    else
    {
      /*unknown sensor, do nothing*/
    }

    //normalize yaw
    while (x_(3)> M_PI) 
    {
      x_(3)-=2.*M_PI;
    }
    while (x_(3)<-M_PI)
    {
      x_(3)+=2.*M_PI;
    }


  }


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1); //augmented sigma points
  
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  PredictAugmentedSigmaPoints(Xsig_aug, delta_t, &Xsig_pred_);

  CalculatePredictionMeanAndCovariance();


}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
  /**
  Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  also to calculate the lidar NIS.
  */
  const VectorXd z = meas_package.raw_measurements_;

  const VectorXd z_pred = H_lidar_ * x_;
  const VectorXd y = z - z_pred;
  const MatrixXd Ht = H_lidar_.transpose();
  const MatrixXd S = H_lidar_ * P_ * Ht + R_lidar_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_lidar_) * P_;
  
  //normalize yaw
  while (x_(3)> M_PI) 
  {
    x_(3)-=2.*M_PI;
  }
  while (x_(3)<-M_PI)
  {
    x_(3)+=2.*M_PI;
  }



}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

   to calculate the radar NIS.
  */
  const int n_z = meas_package.raw_measurements_.size();
    //create vector for mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  // z_pred = ConvCartesianToPolar_CTRV(x_);
  MatrixXd z_sig_pred = MatrixXd(n_z, n_sigma_);
  
  MatrixXd S = MatrixXd(n_z, n_z);

  ConvertPredictedSigmasIntoRadarCoordination(    Xsig_pred_,   /*const MatrixXd& x_sig_pred, */
                                                  &z_pred,      /*VectorXd* z_out, */
                                                  &z_sig_pred,  /*VectorXd* z_sig_out, */
                                                  &S            /*MatrixXd* S_out*/
                                               );

  UpdateState(  meas_package.raw_measurements_,  
                z_pred,    
                z_sig_pred,                      
                S,                               
                &x_,                             
                &P_    
              );



}





void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) {

 
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug << x_, 0, 0;
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.setZero();
  //create augmented mean state

  P_aug.block<n_x_,n_x_>(0,0) = P_;    
  P_aug.block<n_aug_delta_,n_aug_delta_>(n_x_,n_x_) = Q_;    
    
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


}


void UKF::PredictAugmentedSigmaPoints(const MatrixXd& Xsig_aug, double delta_t, MatrixXd* Xsig_out) 
{

  //create a temporary matrix with predicted sigma points as columns
  MatrixXd x_sig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);
  x_sig_pred.setZero();

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
      
      if (fabs(yaw_rate )>0.001)
      {
         px_new       = px + v/yaw_rate*( sin(yaw + yaw_rate*delta_t) - sin(yaw)) + 0.5 *delta_t*delta_t*cos(yaw)*niu_a;
         py_new       = py + v/yaw_rate*(-cos(yaw + yaw_rate*delta_t) + cos(yaw)) + 0.5 *delta_t*delta_t*sin(yaw)*niu_a;
         v_new        = v + delta_t*niu_a;
         
      }
      else
      {
         px_new       = px + v*cos(yaw)*delta_t + 0.5*delta_t*delta_t*cos(yaw)*niu_a; 
         py_new       = px + v*sin(yaw)*delta_t + 0.5*delta_t*delta_t*sin(yaw)*niu_a; 
         v_new        = v + delta_t*niu_a;
      }
      
      yaw_new      = yaw + yaw_rate*delta_t + 0.5*delta_t*delta_t*niu_yaw;
      yaw_rate_new = yaw_rate + delta_t*niu_yaw;

      x_sig_pred.col(i)<<px_new, py_new, v_new, yaw_new, yaw_rate_new;

  }

  //write result
  *Xsig_out = x_sig_pred; //copy assignment. TODO: Make a move assignment

}

void UKF::CalculatePredictionMeanAndCovariance(void)
{


  //TODO: Move weight calc to initialization
  // set weights_
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights_
    weights_(i)  = 0.5/(n_aug_+lambda_);

  }

  //predicted state mean
  x_.setZero();
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_ + weights_(i)*Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.setZero();
  for (int i = 0; i < 2*n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization

    while (x_diff(3)> M_PI) 
    {
      x_diff(3)-=2.*M_PI;
    }
    while (x_diff(3)<-M_PI)
    {
      x_diff(3)+=2.*M_PI;
    }

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ; //Qnote: each x_diff * x_diff.transpose() is a covariance matrix P of that sigma point. average over all 
  }


}




static Eigen::VectorXd ConvCartesianToPolar_CTRV(const Eigen::VectorXd& x)
{
  double px = x[0];
  double py = x[1];
  double v = x[2];
  double yaw = x[3];

  double distance = px*px + py*py;
  VectorXd ret_val(3);
  if(0 != distance)
  {
    double ro = std::sqrt(distance);
    double phi = std::atan2(py,px);
    double vx = v*std::cos(yaw);
    double vy = v*std::sin(yaw);
    
    double ro_dot = (px*vx+py*vy)/(ro);
    ret_val << ro, phi, ro_dot;
  }
  else
  { 
    double ro_dot = 0.0;
    double phi = std::atan2(py,px);

    //if the speed is along x-axis or y-axis

    ro_dot = v;

    ret_val << 0, phi, ro_dot;
    // std::cerr << "ConvPolarToCartesian() - Error - Division by Zero" << std::endl;
  }

  return ret_val;
}




/**
 * Covert the predicted sigma points into the RADAR coordination of 3 states: distance rho, orientation phi, radius speed rho_d. 
 * @param x_sig_pred  the predicted sigma points
 * @param z_out       the predicted mean state converted in the RADAR coordination 
 * @param z_sig_out   the predicted sigma states converted in the RADAR coordination 
 * @param S_out   the measurement covariance matrix
 */
void UKF::ConvertPredictedSigmasIntoRadarCoordination(const MatrixXd& x_sig_pred, 
                                                      VectorXd* z_out, 
                                                      MatrixXd* z_sig_out,
                                                      MatrixXd* S_out) 
{

  int n_sig = x_sig_pred.cols();

  //create matrix for sigma points in measurement space
  MatrixXd z_sig_pts = MatrixXd(n_z_radar_, n_sig);
  z_sig_pts.setZero();
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_radar_);
  z_pred.setZero();
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_radar_,n_z_radar_);  //TODO: replace the intermediate S with *S_out
  S.setZero();

  //transform sigma points into measurement space
  for(int i=0; i<(n_sig); i++)
  {
      //transform sigma points into measurement space
      z_sig_pts.col(i) = ConvCartesianToPolar_CTRV(x_sig_pred.col(i)) /* + noise*/;
      
      //calculate mean predicted measurement
      z_pred += weights_(i)*z_sig_pts.col(i);  //TODO: what is difference from z_pred = ConvCartesianToPolar_CTRV(x_)?????
      

  }
  
  //calculate measurement covariance matrix S
  for(int i=0; i<n_sig; i++)
  {
    VectorXd z_diff = z_sig_pts.col(i) - z_pred;

    //angle normalization 
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S += weights_(i)*z_diff*z_diff.transpose();     
  }
  

  S += R_radar_;

  //write result
  *z_out = z_pred;
  *z_sig_out = z_sig_pts;  //TODO: use move()
  *S_out = S;
}





/**
 * Covert the predicted system state into the sensor measurement state. For example, RADAR space: distance rho, orientation phi, radius speed rho_d. 
 * @param a_z       actual sensor measurement
 * @param a_z_pred  system state in the measurement space/coordination
 * @param a_z_sig   sigma points in measurement space
 * @param a_S       measurement covariance
 * @param x_out     the updated mean states based on the sensor measurement data
 * @param P_out     the updated state covariance 
 */
void UKF::UpdateState(const VectorXd& a_z, 
                      const VectorXd& a_z_pred, 
                      const MatrixXd& a_z_sig, 
                      const MatrixXd& a_S, 
                      VectorXd* x_out, 
                      MatrixXd* P_out
                      ) 
{

  int n_z = a_z.size();// measurement dimension

  //create temporary vector for predicted state mean
  VectorXd x = VectorXd(n_x_);
  x = *x_out;


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();

  //calculate cross correlation matrix
  for(int i=0; i<n_sigma_; i++)
  {
    VectorXd x_sig_diff(n_x_);
    x_sig_diff << Xsig_pred_.col(i) - x;

    //angle normalization
    while (x_sig_diff(3)> M_PI) x_sig_diff(3)-=2.*M_PI;
    while (x_sig_diff(3)<-M_PI) x_sig_diff(3)+=2.*M_PI;


    VectorXd z_sig_diff(n_z);
    z_sig_diff << a_z_sig.col(i) - a_z_pred;
       
    //angle normalization 
    //TODO: add sensor-specific normalization function under measurement z.  
    while (z_sig_diff(1)> M_PI) z_sig_diff(1)-=2.*M_PI;
    while (z_sig_diff(1)<-M_PI) z_sig_diff(1)+=2.*M_PI;

    //cross-correlation between sigma points in state space and measurement space
    Tc += weights_(i)*x_sig_diff*z_sig_diff.transpose();
      

  }

  //calculate Kalman gain K;
  MatrixXd K(n_x_, n_z);
  K.setZero();

  K << Tc*a_S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff = a_z - a_z_pred;
  //angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

  x += K*z_diff;


  *P_out -= K*a_S*K.transpose();

  //write result
  *x_out = x;

}


