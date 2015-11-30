#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
#include <map>
#include <vector>
#include <functional>

#ifndef EIGEN_HELPERS_HPP
#define EIGEN_HELPERS_HPP

namespace std
{
    template <>
    struct hash<Eigen::Vector3d>
    {
        std::size_t operator()(const Eigen::Vector3d& vector) const
        {
            using std::size_t;
            using std::hash;
            return ((std::hash<double>()(vector.x()) ^ (std::hash<double>()(vector.y()) << 1) >> 1) ^ (std::hash<double>()(vector.z()) << 1));
        }
    };
}

namespace EigenHelpers
{
    ////////////////////////////////////////////////////////////////////////////
    // Typedefs for aligned STL containers using Eigen types
    ////////////////////////////////////////////////////////////////////////////

    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> VectorVector3f;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VectorVector3d;
    typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> VectorVector4f;
    typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> VectorVector4d;
    typedef std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf>> VectorQuaternionf;
    typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> VectorQuaterniond;
    typedef std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>> VectorAffine3f;
    typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> VectorAffine3d;
    typedef std::map<std::string, Eigen::Vector3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3f>>> MapStringVector3f;
    typedef std::map<std::string, Eigen::Vector3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> MapStringVector3d;
    typedef std::map<std::string, Eigen::Vector4f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4f>>> MapStringVector4f;
    typedef std::map<std::string, Eigen::Vector4d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector4d>>> MapStringVector4d;
    typedef std::map<std::string, Eigen::Quaternionf, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaternionf>>> MapStringQuaternionf;
    typedef std::map<std::string, Eigen::Quaterniond, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Quaterniond>>> MapStringQuaterniond;
    typedef std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>> MapStringAffine3f;
    typedef std::map<std::string, Eigen::Affine3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3d>>> MapStringAffine3d;

    inline bool Equal(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        if (v1.x() == v2.x() && v1.y() == v2.y() && v1.z() == v2.z())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    inline bool CloseEnough(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const double threshold)
    {
        double real_threshold = fabs(threshold);
        if (fabs(v1.x() - v2.x()) > real_threshold)
        {
            return false;
        }
        if (fabs(v1.y() - v2.y()) > real_threshold)
        {
            return false;
        }
        if (fabs(v1.z() - v2.z()) > real_threshold)
        {
            return false;
        }
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Interpolation functions
    ////////////////////////////////////////////////////////////////////////////

    inline double Interpolate(const double p1, const double p2, const double ratio)
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
        return ((p1 * (1.0 - real_ratio)) + (p2 * real_ratio));
    }

    inline std::vector<double> Interpolate(const std::vector<double>& v1, const std::vector<double>& v2, const double ratio)
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Safety check inputs
        size_t len = v1.size();
        if (len == v2.size())
        {
            std::cerr << "Vectors to interpolate are different sizes" << std::endl;
            return std::vector<double>();
        }
        // Interpolate
        // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
        std::vector<double> interped(len, 0);
        for (size_t idx = 0; idx < len; idx++)
        {
            interped[idx] = ((v1[idx] * (1.0 - real_ratio)) + (v2[idx] * real_ratio));
        }
        return interped;
    }

    inline Eigen::Quaterniond Interpolate(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const double ratio)
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        return q1.slerp(real_ratio, q2);
    }

    inline Eigen::Vector3d Interpolate(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2, const double ratio)
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
        return ((v1 * (1.0 - real_ratio)) + (v2 * real_ratio));
    }

    inline Eigen::Affine3d Interpolate(const Eigen::Affine3d& t1, const Eigen::Affine3d& t2, const double ratio)
    {
        // Safety check ratio
        double real_ratio = ratio;
        if (real_ratio < 0.0)
        {
            real_ratio = 0.0;
            std::cerr << "Interpolation ratio < 0.0, set to 0.0" << std::endl;
        }
        else if (real_ratio > 1.0)
        {
            real_ratio = 1.0;
            std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
        }
        // Interpolate
        Eigen::Vector3d v1 = t1.translation();
        Eigen::Quaterniond q1(t1.rotation());
        Eigen::Vector3d v2 = t2.translation();
        Eigen::Quaterniond q2(t2.rotation());
        Eigen::Vector3d vint = Interpolate(v1, v2, real_ratio);
        Eigen::Quaterniond qint = Interpolate(q1, q2, real_ratio);
        Eigen::Affine3d tint = ((Eigen::Translation3d)vint) * qint;
        return tint;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Distance functions
    ////////////////////////////////////////////////////////////////////////////

    inline double Distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        return (v2 - v1).norm();
    }

    inline double Distance(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2)
    {
        double dq = fabs((q1.w() * q2.w()) + (q1.x() * q2.x()) + (q1.y() * q2.y()) + (q1.z() * q2.z()));
        if (dq < (1.0 - std::numeric_limits<double>::epsilon()))
        {
            return acos(2.0 * (dq * dq) - 1.0);
        }
        else
        {
            return 0.0;
        }
    }

    inline double Distance(const Eigen::Affine3d& t1, const Eigen::Affine3d& t2)
    {
        Eigen::Vector3d v1 = t1.translation();
        Eigen::Quaterniond q1(t1.rotation());
        Eigen::Vector3d v2 = t2.translation();
        Eigen::Quaterniond q2(t2.rotation());
        double vdist = Distance(v1, v2);
        double qdist = Distance(q1, q2);
        return vdist + qdist;
    }

    inline double Distance(const std::vector<double>& p1, const std::vector<double>& p2)
    {
        if (p1.size() == p2.size())
        {
            double distance = 0.0;
            for (size_t idx = 0; idx < p1.size(); idx++)
            {
                distance += (p2[idx] - p1[idx]) * (p2[idx] - p1[idx]);
            }
            return sqrt(distance);
        }
        else
        {
            return INFINITY;
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Vector3d StdVectorDoubleToEigenVector3d(const std::vector<double>& vector)
    {
        if (vector.size() != 3)
        {
            std::cerr << "Vector3d source vector is not 3 elements in size" << std::endl;
            assert(false);
        }
        Eigen::Vector3d eigen_vector(vector[0], vector[1], vector[2]);
        return eigen_vector;
    }

    inline std::vector<double> EigenVector3dToStdVectorDouble(const Eigen::Vector3d& point)
    {
        return std::vector<double>{point.x(), point.y(), point.z()};
    }

    // Takes <x, y, z, w> as is the ROS custom!
    inline Eigen::Quaterniond StdVectorDoubleToEigenQuaterniond(const std::vector<double>& vector)
    {
        if (vector.size() != 4)
        {
            std::cerr << "Quaterniond source vector is not 4 elements in size" << std::endl;
            assert(false);
        }
        Eigen::Quaterniond eigen_quaternion(vector[3], vector[0], vector[1], vector[2]);
        return eigen_quaternion;
    }

    // Returns <x, y, z, w> as is the ROS custom!
    inline std::vector<double> EigenQuaterniondToStdVectorDouble(const Eigen::Quaterniond& quat)
    {
        return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
    }

    ////////////////////////////////////////////////////////////////////////////
    // Averaging functions
    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Vector3d AverageEigenVector3d(const EigenHelpers::VectorVector3d& vectors, const std::vector<double>& weights=std::vector<double>())
    {
        assert(vectors.size() > 0);
        assert((weights.size() == vectors.size()) || (weights.size() == 0));
        if (vectors.size() == 1)
        {
            return vectors[0];
        }
        // Get the weights
        bool use_weights = false;
        double sum_weights = 0.0;
        if (weights.size() == vectors.size())
        {
            use_weights = true;
            for (size_t idx = 0; idx < weights.size(); idx++)
            {
                sum_weights += fabs(weights[idx]);
            }
            assert(sum_weights > 0.0);
        }
        else
        {
            sum_weights = (double)vectors.size();
        }
        // Do the weighted averaging
        Eigen::Vector3d sum_vector(0.0, 0.0, 0.0);
        for (size_t idx = 0; idx < vectors.size(); idx++)
        {
            double ew = 1.0;
            if (use_weights)
            {
                ew = fabs(weights[idx]);
            }
            const Eigen::Vector3d& prev_sum = sum_vector;
            const Eigen::Vector3d& current = vectors[idx];
            sum_vector = prev_sum + ((ew / sum_weights) * (current - prev_sum));
        }
        return sum_vector;
    }

    /*
     * Implementation of method described in (http://stackoverflow.com/a/27410865)
     * See paper at (http://www.acsu.buffalo.edu/~johnc/ave_quat07.pdf) for full explanation
     */
    inline Eigen::Quaterniond AverageEigenQuaterniond(const EigenHelpers::VectorQuaterniond& quaternions, const std::vector<double>& weights=std::vector<double>())
    {
        assert(quaternions.size() > 0);
        assert((weights.size() == quaternions.size()) || (weights.size() == 0));
        if (quaternions.size() == 1)
        {
            return quaternions[0];
        }
        bool use_weights = false;
        if (weights.size() == quaternions.size())
        {
            use_weights = true;
        }
        // Build the averaging matrix
        Eigen::MatrixXd q_matrix(4, quaternions.size());
        for (size_t idx = 0; idx < quaternions.size(); idx++)
        {
            double ew = 1.0;
            if (use_weights)
            {
                ew = fabs(weights[idx]);
            }
            const Eigen::Quaterniond& q = quaternions[idx];
            q_matrix.col(idx) << ew * q.w(), ew * q.x(), ew * q.y(), ew * q.z();
        }
        // Make the matrix square
        Eigen::Matrix<double, 4, 4> qqtranspose_matrix = q_matrix * q_matrix.transpose();
        // Compute the eigenvectors and eigenvalues of the qqtranspose matrix
        Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> solver(qqtranspose_matrix);
        Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvalueType eigen_values = solver.eigenvalues();
        Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>>::EigenvectorsType eigen_vectors = solver.eigenvectors();
        // Extract the eigenvector corresponding to the largest eigenvalue
        double max_eigenvalue = -INFINITY;
        int64_t max_eigenvector_index = -1;
        for (size_t idx = 0; idx < 4; idx++)
        {
            const double current_eigenvalue = eigen_values(idx).real();
            if (current_eigenvalue > max_eigenvalue)
            {
                max_eigenvalue = current_eigenvalue;
                max_eigenvector_index = idx;
            }
        }
        assert(max_eigenvector_index >= 0);
        // Note that these are already normalized!
        const Eigen::Vector4cd best_eigenvector = eigen_vectors.col(max_eigenvector_index);
        // Convert back into a quaternion
        const Eigen::Quaterniond average_q(best_eigenvector(0).real(), best_eigenvector(1).real(), best_eigenvector(2).real(), best_eigenvector(3).real());
        return average_q;
    }

    inline Eigen::Affine3d AverageEigenAffine3d(const EigenHelpers::VectorAffine3d& transforms, const std::vector<double>& weights=std::vector<double>())
    {
        assert(transforms.size() > 0);
        assert((weights.size() == transforms.size()) || (weights.size() == 0));
        if (transforms.size() == 1)
        {
            return transforms[0];
        }
        // Extract components
        EigenHelpers::VectorVector3d translations(transforms.size());
        EigenHelpers::VectorQuaterniond rotations(transforms.size());
        for (size_t idx = 0; idx < transforms.size(); idx++)
        {
            translations[idx] = transforms[idx].translation();
            rotations[idx] = Eigen::Quaterniond(transforms[idx].rotation());
        }
        // Average
        const Eigen::Vector3d average_translation = AverageEigenVector3d(translations, weights);
        const Eigen::Quaterniond average_rotation = AverageEigenQuaterniond(rotations, weights);
        // Make the average transform
        const Eigen::Affine3d average_transform = (Eigen::Translation3d)average_translation * average_rotation;
        return average_transform;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Other auxiliary functions
    ////////////////////////////////////////////////////////////////////////////

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

#endif // EIGEN_HELPERS_HPP
