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

// extern "C" {
//   int vector_sub(double *v1, double *v2, double *result){
//     result[0] = v1[0] - v2[0];
//     result[1] = v1[1] - v2[1];
//     result[2] = v1[2] - v2[2];
//     return 0;
//   }
// }