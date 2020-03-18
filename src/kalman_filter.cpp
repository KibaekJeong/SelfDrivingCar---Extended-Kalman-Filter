#include "kalman_filter.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

Tools KalmanFilter::tools_ =  Tools();


void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    UpdateCommon(y,H_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
    MatrixXd Hj = tools_.CalculateJacobian(x_);

    double rho, phi, rho_dot;
    rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    phi = atan2(x_(1),x_(0));

    if (fabs(rho)<0.0001){
        rho_dot = 0;
    }
    else{
        rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
    }
    VectorXd polar_v = VectorXd(3);
    polar_v << rho,phi,rho_dot;

    VectorXd y = z- polar_v;
    // Normalize

    while(y[1]>M_PI) y[1] -=2.*M_PI;
    while(y[1]<-M_PI) y[1] +=2.*M_PI;

    UpdateCommon(y,Hj);
}
void KalmanFilter::UpdateCommon(const VectorXd &y, const MatrixXd &H){
    MatrixXd S = H * P_ * H.transpose() + R_;
    MatrixXd PHt = P_ * H.transpose();
    MatrixXd K = PHt * S.inverse();

    //New estimate
    x_= x_ +(K*y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size,x_size);
    P_ = (I-K*H)*P_;
}
