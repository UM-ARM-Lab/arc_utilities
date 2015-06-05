#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include "arc_utilities/eigen_pinv.hpp"

using namespace EIGEN_PINV;
// Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
// see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
Eigen::MatrixXd pinv( const Eigen::MatrixXd &b, double rcond )
{
    // TODO: Figure out why it wants fewer rows than columns
//    if ( a.rows()<a.cols() )
//        return false;
    bool flip = false;
    Eigen::MatrixXd a;
    if( a.rows() < a.cols() )
    {
        a = b.transpose();
        flip = true;
    }
    else
        a = b;

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
    svdA.compute( a, Eigen::ComputeFullU | Eigen::ComputeThinV );

    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular = svdA.singularValues();

    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::VectorXd vPseudoInvertedSingular( svdA.matrixV().cols() );

    for (int iRow=0; iRow<vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow)) <= rcond ) // Todo : Put epsilon in parameter
        {
            vPseudoInvertedSingular(iRow)=0.;
        }
        else
            vPseudoInvertedSingular(iRow)=1./vSingular(iRow);
    }

    // A little optimization here
    Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block( 0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols() );

    // Pseudo-Inversion : V * S * U'
    Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;

    if( flip )
    {
        a = a.transpose();
        a_pinv = a_pinv.transpose();
    }

    return a_pinv;
}
