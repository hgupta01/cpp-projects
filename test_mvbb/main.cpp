#include <iostream>
#include "ApproxMVBB/ComputeApproxMVBB.hpp"

int main(int argc, char** argv)
{
    unsigned int nPoints = 100000;

    std::cout << "Sample " << nPoints << " points in unite cube (coordinates are in world coordinate system `I` ) " << std::endl;
    ApproxMVBB::Matrix3Dyn points(3, nPoints);
    points.setRandom();

    ApproxMVBB::OOBB oobb = ApproxMVBB::approximateMVBB(points,
                                                        0.001,
                                                        500,
                                                        5, /*increasing the grid size decreases speed */
                                                        0,
                                                        5);

    std::cout << "Computed OOBB: " << std::endl
              << "---> lower point in OOBB coordinate system: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB coordinate system: " << oobb.m_maxPoint.transpose() << std::endl
              << "---> coordinate transformation A_IK matrix from OOBB coordinate system `K`  "
                 "to world coordinate system `I` "
              << std::endl
              << oobb.m_q_KI.matrix() << std::endl
              << "---> this is also the rotation matrix R_KI  which turns the "
                 "world coordinate system `I`  into the OOBB coordinate system `K` "
              << std::endl
              << std::endl;

    // To make all points inside the OOBB :
    ApproxMVBB::Matrix33 A_KI = oobb.m_q_KI.matrix().transpose();  // faster to store the transformation matrix first
    auto size                 = points.cols();
    for(unsigned int i = 0; i < size; ++i)
    {
        oobb.unite(A_KI * points.col(i));
    }

    // To make the box have a minimum extent of greater 0.1:
    // see also oobb.expandToMinExtentRelative(...)
    oobb.expandToMinExtentAbsolute(0.1);

    std::cout << "OOBB with all point included: " << std::endl
              << "---> lower point in OOBB coordinate system: " << oobb.m_minPoint.transpose() << std::endl
              << "---> upper point in OOBB coordinate system: " << oobb.m_maxPoint.transpose() << std::endl;

    return 0;

}