#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Initialize Variables
  n_x_ = 5;

  n_aug_ = 7;

  is_initialized_ = false;

  lambda_ = 3 - n_aug_;


  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  weights_ = VectorXd(2*n_aug_+1);







}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
    if (!is_initialized_)
    {

        if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            x_.head(2)=meas_package.raw_measurements_;

        }
        else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            x_(2)= 0;  //meas_package.raw_measurements_(0);
            x_(3)= 0; //meas_package.raw_measurements_(1);
            x_(4)= 0; //meas_package.raw_measurements_(2);

            P_<< 0.5,0,0,0,0,
                    0,0.5,0,0,0,
                    0,0,0.5,0,0,
                    0,0,0,0.5,0,
                    0,0,0,0,0.5;

            // set weights
            double weight_0 = lambda_/(lambda_+n_aug_);
            weights_(0) = weight_0;
            for (int i=1; i<2*n_aug_+1; ++i) // 2n+1 weight
            {
                double weight = 0.5/(n_aug_+lambda_);
                weights_(i) = weight;
            }

            is_initialized_ = true;


            Prediction(1.0 / 30.0);

            std::cout << x_ << std::endl;
        }

    }
    else
    {

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            UpdateLidar(meas_package);


        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            std::cout << meas_package.raw_measurements_ << std::endl;
            UpdateRadar(meas_package);
            Prediction(1.0 / 30.0);

        }

    }


}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);
    AugmentedSigmaPoints(Xsig_aug);  //Function needs state and covariance
    SigmaPointPrediction(Xsig_aug, delta_t); //Function need Augmented Sigma Points but modified outputs a predicted standar sigma point (n_x, 2*n_aug +1)
    PredictMeanAndCovariance();


}


/*
void UKF::GenerateSigmaPoints(Eigen::MatrixXd* Xsig_out) {

    // set state dimension
    int n_x = 5;

    // define spreading parameter
    double lambda = 3 - n_x;

    // set example state
    VectorXd x = VectorXd(n_x);
    x <<   5.7441,
            1.3800,
            2.2049,
            0.5015,
            0.3528;

    // set example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
            -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
            0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
            -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
            -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);

    // calculate square root of P
    MatrixXd A = P.llt().matrixL();



    // set first column of sigma point matrix
    Xsig.col(0) = x;

    // set remaining sigma points
    for (int i = 0; i < n_x; ++i) {
        Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);
        Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);
    }


    // print result
    // std::cout << "Xsig = " << std::endl << Xsig << std::endl;

    // write result
    *Xsig_out = Xsig;
}

*/



void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_out)
{

    // create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);

    // create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    // create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    // create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; ++i)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    // write result
    Xsig_out = Xsig_aug;


}

void UKF::SigmaPointPrediction(Eigen::MatrixXd& Xsig_aug, double delta_t)
{

    // predict sigma points
    for (int i = 0; i< 2*n_aug_+1; ++i)
    {
        // extract values for better readability
        double p_x = Xsig_aug(0,i);
        double p_y = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_yawdd = Xsig_aug(6,i);

        // predicted state values
        double px_p, py_p;

        // avoid division by zero
        if (fabs(yawd) > 0.001)
        {
            px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
            py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
        }
        else
        {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        // add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        // write predicted sigma point into right column
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawd_p;
    }

}

void UKF::PredictMeanAndCovariance()
{

    // predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {  // iterate over sigma points
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }

    // print result
    std::cout << "Predicted state" << std::endl;
    std::cout << x_ << std::endl;
    std::cout << "Predicted covariance matrix" << std::endl;
    std::cout << P_ << std::endl;

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

    MatrixXd S;


    VectorXd z = meas_package.raw_measurements_;

    int n_z = 2;

    /**
     * Calculate S = H P H.transpose() + R
     */
    MatrixXd H = MatrixXd(n_z, n_x_);
    MatrixXd R = MatrixXd(n_z,n_z);

    H << 1,0,0,0,0,
         0,1,0,0,0;

    R << std_laspx_*std_laspx_, 0,
         0, std_laspy_*std_laspy_;


    VectorXd z_pred = H * x_;


    S = H*P_*H.transpose() + R;


    VectorXd y = z - z_pred;

    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd K = PHt * Si;


    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;

    // print result

    std::cout << "Updated state x: " << std::endl << x_ << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;


}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
   VectorXd z_pred;
   MatrixXd S;
   MatrixXd Zsig;
   PredictRadarMeasurement(z_pred, S, Zsig);

   VectorXd z = meas_package.raw_measurements_;

   int n_z = 3;

    // create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    /**
     * Student part begin
     */

    // calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Kalman gain K;
    MatrixXd K = Tc * S.inverse();

    // residual
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S*K.transpose();


    // print result
    std::cout << "Updated state x: " << std::endl << x_ << std::endl;
    std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;


}


void UKF::PredictRadarMeasurement(VectorXd& z_out, MatrixXd& S_out, MatrixXd& Z_out)
{

    // set measurement dimension, radar can measure r, phi, and r_dot
    int n_z = 3;

    // create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

    // mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);

    // measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);

    /**
     * Student part begin
     */

    // transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // extract values for better readability
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;

        // measurement model
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
        Zsig(1,i) = atan2(p_y,p_x);                                // phi
        Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
    }


    // mean predicted measurement
    z_pred.fill(0.0);
    for (int i=0; i < 2*n_aug_+1; ++i) {
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }



    // innovation covariance matrix S
    S.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
        // residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    // add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<  std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0,std_radrd_*std_radrd_;
    S = S + R;


    // write result
    z_out = z_pred;
    S_out = S;
    Z_out = Zsig;
}


