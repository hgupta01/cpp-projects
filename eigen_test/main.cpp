#include <iostream>
#include <eigen3/Eigen/Dense>

int main(){
  Eigen::MatrixXd mat;
  mat.resize(2,5);
  mat << 0,1,2,3,4,5,6,7,8,9;

  Eigen::Vector2d vec;
  vec << 5, 5;


  std::cout <<(mat.colwise()-vec).colwise().norm();
}
