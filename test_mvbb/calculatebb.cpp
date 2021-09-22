#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

void matrix2Array(const Eigen::MatrixXd transform, double *array) {
  array[0] = transform(0, 0);
  array[1] = transform(0, 1);
  array[2] = transform(0, 2);

  array[3] = transform(1, 0);
  array[4] = transform(1, 1);
  array[5] = transform(1, 2);

  array[6] = transform(2, 0);
  array[7] = transform(2, 1);
  array[8] = transform(2, 2);

  array[9] = transform(3, 0);
  array[10] = transform(3, 1);
  array[11] = transform(3, 2);

  array[12] = transform(4, 0);
  array[13] = transform(4, 1);
  array[14] = transform(4, 2);
}


extern "C" 
{
  int calculateBB(const double *pc, const size_t n_points, double *result) {
    ApproxMVBB::Matrix3Dyn points_(3, n_points);

    size_t index = 0;
    for (size_t i = 0; i < n_points; ++i){
      points_.col(i) << pc[index], pc[index + 1], pc[index + 2];
      index += 3;
    }

    ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points_,
                                                        0.001,
                                                        500,
                                                        5, /*increasing the grid size decreases speed */
                                                        0,
                                                        5);

    // To make all points inside the OOBB :
    ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();  // faster to store the transformation matrix first
    auto size                 = points_.cols();
    for(unsigned int i = 0; i < size; ++i)
    {
        oobb.unite(A_KI * points_.col(i));
    }
    oobb.expandToMinExtentAbsolute(0.01);

    Eigen::Matrix<double, 5, 3> rmat;
    rmat.row(0) = oobb.m_minPoint.transpose();
    rmat.row(1) = oobb.m_maxPoint.transpose();
    rmat.block<3, 3>(2, 0) = oobb.m_q_KI.matrix();
    
    matrix2Array(rmat, result);

    return 0;
  }
}



