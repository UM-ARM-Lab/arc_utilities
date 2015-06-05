#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/SVD>

#ifndef EIGEN_PINV_HPP
#define EIGEN_PINV_HPP

namespace EIGEN_PINV
{
    Eigen::MatrixXd pinv( const Eigen::MatrixXd &b, const double rcond);
}

#endif // EIGEN_PINV_HPP
