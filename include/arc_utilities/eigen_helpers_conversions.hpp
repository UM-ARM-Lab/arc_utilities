#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
#include <arc_utilities/pretty_print.hpp>
#include <functional>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#ifndef EIGEN_HELPERS_CONVERSIONS_HPP
#define EIGEN_HELPERS_CONVERSIONS_HPP

namespace EigenHelpersConversions
{
    inline Eigen::Vector3d GeometryPointToEigenVector3d(const geometry_msgs::Point& point)
    {
        Eigen::Vector3d eigen_point(point.x, point.y, point.z);
        return eigen_point;
    }

    inline geometry_msgs::Point EigenVector3dToGeometryPoint(const Eigen::Vector3d& point)
    {
        geometry_msgs::Point geom_point;
        geom_point.x = point.x();
        geom_point.y = point.y();
        geom_point.z = point.z();
        return geom_point;
    }

    inline Eigen::Vector3d GeometryVector3ToEigenVector3d(const geometry_msgs::Vector3& vector)
    {
        Eigen::Vector3d eigen_vector(vector.x, vector.y, vector.z);
        return eigen_vector;
    }

    inline geometry_msgs::Vector3 EigenVector3dToGeometryVector3(const Eigen::Vector3d& vector)
    {
        geometry_msgs::Vector3 geom_vector;
        geom_vector.x = vector.x();
        geom_vector.y = vector.y();
        geom_vector.z = vector.z();
        return geom_vector;
    }

    inline Eigen::Quaterniond GeometryQuaternionToEigenQuaterniond(const geometry_msgs::Quaternion& quat)
    {
        Eigen::Quaterniond eigen_quaternion(quat.w, quat.x, quat.y, quat.z);
        return eigen_quaternion;
    }

    inline geometry_msgs::Quaternion EigenQuaterniondToGeometryQuaternion(const Eigen::Quaterniond& quat)
    {
        geometry_msgs::Quaternion geom_quaternion;
        geom_quaternion.w = quat.w();
        geom_quaternion.x = quat.x();
        geom_quaternion.y = quat.y();
        geom_quaternion.z = quat.z();
        return geom_quaternion;
    }

    inline Eigen::Affine3d GeometryPoseToEigenAffine3d(const geometry_msgs::Pose& pose)
    {
        Eigen::Translation3d trans(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Affine3d eigen_pose = trans * quat;
        return eigen_pose;
    }

    inline geometry_msgs::Pose EigenAffine3dToGeometryPose(const Eigen::Affine3d& transform)
    {
        Eigen::Vector3d trans = transform.translation();
        Eigen::Quaterniond quat(transform.rotation());
        geometry_msgs::Pose geom_pose;
        geom_pose.position.x = trans.x();
        geom_pose.position.y = trans.y();
        geom_pose.position.z = trans.z();
        geom_pose.orientation.w = quat.w();
        geom_pose.orientation.x = quat.x();
        geom_pose.orientation.y = quat.y();
        geom_pose.orientation.z = quat.z();
        return geom_pose;
    }

    inline Eigen::Affine3d GeometryTransformEigenAffine3d(const geometry_msgs::Transform& transform)
    {
        Eigen::Translation3d trans(transform.translation.x, transform.translation.y, transform.translation.z);
        Eigen::Quaterniond quat(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        Eigen::Affine3d eigen_transform = trans * quat;
        return eigen_transform;
    }

    inline geometry_msgs::Transform EigenAffine3dToGeometryTransform(const Eigen::Affine3d& transform)
    {
        Eigen::Vector3d trans = transform.translation();
        Eigen::Quaterniond quat(transform.rotation());
        geometry_msgs::Transform geom_transform;
        geom_transform.translation.x = trans.x();
        geom_transform.translation.y = trans.y();
        geom_transform.translation.z = trans.z();
        geom_transform.rotation.w = quat.w();
        geom_transform.rotation.x = quat.x();
        geom_transform.rotation.y = quat.y();
        geom_transform.rotation.z = quat.z();
        return geom_transform;
    }

    inline Eigen::Matrix3Xd VectorGeometryPointToEigenMatrix3Xd(const std::vector<geometry_msgs::Point>& vector_geo)
    {
        Eigen::Matrix3Xd eigen_matrix = Eigen::MatrixXd(3, vector_geo.size());
        for (size_t ind = 0; ind < vector_geo.size(); ind++)
        {
            eigen_matrix.block<3,1>(0, ind) = GeometryPointToEigenVector3d(vector_geo[ind]);
        }
        return eigen_matrix;
    }

    inline VectorAffine3d VectorGeometryPoseToVectorAffine3d(const std::vector<geometry_msgs::Pose>& vector_geo)
    {
        VectorAffine3d vector_eigen(vector_geo.size());
        for (size_t ind = 0; ind < vector_geo.size(); ind++)
        {
            vector_eigen[ind] = GeometryPoseToEigenAffine3d(vector_geo[ind]);
        }
        return vector_eigen;
    }

=======
>>>>>>> 63bb6be8c170df3ce2956596eb24015460cfc79a
    inline double SuggestedRcond()
    {
        return 0.001;
    }

    // Derived from code by Yohann Solaro ( http://listengine.tuxfamily.org/lists.tuxfamily.org/eigen/2010/01/msg00187.html )
    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
    inline Eigen::MatrixXd Pinv(const Eigen::MatrixXd& b, const double rcond)
    {
        bool flip = false;
        Eigen::MatrixXd a;
        if (a.rows() < a.cols())
        {
            a = b.transpose();
            flip = true;
        }
        else
        {
            a = b;
        }
        // SVD
        Eigen::JacobiSVD<Eigen::MatrixXd> svdA;
        svdA.compute(a, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType vSingular = svdA.singularValues();
        // Build a diagonal matrix with the Inverted Singular values
        // The pseudo inverted singular matrix is easy to compute :
        // is formed by replacing every nonzero entry by its reciprocal (inversing).
        Eigen::VectorXd vPseudoInvertedSingular(svdA.matrixV().cols());
        for (int iRow = 0; iRow < vSingular.rows(); iRow++)
        {
            if (fabs(vSingular(iRow)) <= rcond) // Todo : Put epsilon in parameter
            {
                vPseudoInvertedSingular(iRow)= 0.0;
            }
            else
            {
                vPseudoInvertedSingular(iRow) = 1.0 / vSingular(iRow);
            }
        }
        // A little optimization here
        Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());
        // Pseudo-Inversion : V * S * U'
        Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;
        // Flip back if need be
        if (flip)
        {
            a = a.transpose();
            a_pinv = a_pinv.transpose();
        }
        return a_pinv;
    }
}

#endif // EIGEN_HELPERS_CONVERSIONS_HPP
