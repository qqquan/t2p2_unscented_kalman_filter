#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if( 
        (0==estimations.size())
        ||
        (0==ground_truth.size())
        ||
        (estimations.size() != ground_truth.size())
       )
    {
        std::cerr << "Error in inputs of estimation or ground_truth!" << std::endl;
    }
    
  //accumulate squared residuals
  VectorXd residual_sum(4);
  residual_sum << 0,0,0,0;
  for(unsigned int i=0; i < estimations.size(); ++i){
        VectorXd diff  = (estimations[i] - ground_truth[i]);
        VectorXd squared_residual = diff.array()*diff.array();
        residual_sum += squared_residual;
  }

  //calculate the mean
  VectorXd residual_mean = residual_sum/estimations.size();

  //calculate the squared root
  rmse = residual_mean.array().sqrt();


  //return the result
  return rmse;

}
