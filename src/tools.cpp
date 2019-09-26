#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   *
   */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    //check estimation and ground truth value data
    if (estimations.size() != ground_truth.size() || estimations.size()==0){
        cout<< "Invalid estimation or ground_truth data"<<endl;
     return rmse;
    }
    //calculate rmse
    for (int i =0;i<estimations.size();i++){
        VectorXd residual = estimations[i] - ground_truth[i];
        residual = residual.array()*residual.array();
        rmse += residual;
    }
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    MatrixXd Hj(3,4);
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);
    // calculate set of terms
    float c1 = px*px+py*py;
    float c2 = sqrt(c1);
    float c3 =(c1*c2);
    // check division by zero
    if (fabs(c1)<0.0001){
        cout<<"Error - Jacobian - Division by Zero"<<endl;
        return Hj;
    }
    // calculate jacobian
    Hj << (px/c2),(py/c2),0,0,
        -(py/c1),(px/c1),0,0,
        py*(vx*py-vy*px)/c3, px*(vy*px-vx*py)/c3, px/c2, py/c2;
    return Hj;
}
