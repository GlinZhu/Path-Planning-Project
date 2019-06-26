#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include <math.h>
//#include "helpers.h"
#include "spline.h"
// for convenience
using std::string;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

vector<double> JMT(vector<double> start, vector<double> end, double T){
   MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
	
}
double EvalTraj(std::vector<double> coeffs, double x) {
  double y = 0;
  for (int i = 0; i < coeffs.size(); ++i) {
    y += coeffs[i] * pow(x, i);
  }
  return y;
}

/**
 * Differentiate a polynomial with form y = a0 + a1*x + a2*x^2 + ... to get
 * the derivative polynomial's coefficients
 */
vector<double> DiffTraj(std::vector<double> coeffs) {
  std::vector<double> diff_coeffs;
  for (int i = 1; i < coeffs.size(); ++i) {
    diff_coeffs.push_back(i*coeffs[i]);
  }
  return diff_coeffs;
}


#endif
