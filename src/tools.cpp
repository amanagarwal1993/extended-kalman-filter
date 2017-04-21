#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    // Always check inputs!
    if ((estimations.size() != ground_truth.size()) | (estimations.size() == 0)) {
        std::cout << "Invalid size of estimations data. Error!"<<"/n";
        return rmse;
    }
    
    
    for (unsigned int i = 0; i < estimations.size(); i++) {
        VectorXd error = estimations[i] - ground_truth[i];
        error = error.array() * error.array();
        
        rmse += error;
    };
    
    rmse = rmse / (estimations.size());
    
    return rmse;
    
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
   // If x_state = [px py vx vy], then we can readily calculate the Jacobian simply by plugging in a formula.
  */
    MatrixXd Jacobian(3,4);
    
    // Always check inputs!
    if (x_state.size() != 4) {
        std::cout << "Invalid arguments, check x_state again."<<"/n";
        return Jacobian;
    };
    
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];
    
    float root = sqrtf(px*px + py*py);
    float root2 = root * root;
    float root3 = root2 * root;
    
    if (root < 0.0001) {
        std::cout << "Error: dividing by zero"<<"/n";
        return Jacobian;
    };
    
    Jacobian << (px/root), (py/root), 0, 0,
                (-py/root2), (px/root2), 0, 0,
                py*(vx*py-px*vy)/root3, px*(vy*px-py*vx)/root3, px/root, py/root;
    
    return Jacobian;
}
