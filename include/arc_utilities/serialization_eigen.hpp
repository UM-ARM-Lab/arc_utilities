#ifndef SERIALIZATION_EIGEN_HPP
#define SERIALIZATION_EIGEN_HPP

#include "arc_utilities/serialization.hpp"
#include "arc_utilities/eigen_helpers.hpp"

namespace arc_utilities
{
    //////////////////////////////////////////// WARNING!!! ////////////////////////////////////////////////////////////
    // These functions have only been tested with "normal" matrix types: Vector3d, VectorXd, Matrix3d, Matrix Xd
    //////////////////////////////////////////// WARNING!!! ////////////////////////////////////////////////////////////

    template<typename MatrixType>
    inline uint64_t SerializeEigen(
            const MatrixType& eigen,
            std::vector<uint8_t>& buffer)
    {
        uint64_t bytes_written = 0;
        for (size_t idx = 0; idx < eigen.size(); ++idx)
        {
            bytes_written += SerializeFixedSizePOD(eigen.data()[idx], buffer);
        }
        return bytes_written;
    }

    template<typename MatrixType>
    inline std::pair<MatrixType, uint64_t> DeserializeFixedSizeEigen(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(MatrixType);
        MatrixType eigen;
        uint64_t bytes_read = 0;
        for (size_t idx = 0; idx < eigen.size(); ++idx)
        {
            const auto deserialized = DeserializeFixedSizePOD<typename MatrixType::Scalar>(buffer, current + bytes_read);
            eigen.data()[idx] = deserialized.first;
            bytes_read += deserialized.second;
        }
        return {eigen, bytes_read};
    }

    template<typename MatrixType>
    inline std::pair<MatrixType, uint64_t> DeserializeDynamicSizeEigen(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const int64_t rows,
            const int64_t cols)
    {
        MatrixType eigen(rows, cols);
        uint64_t bytes_read = 0;
        for (size_t idx = 0; idx < eigen.size(); ++idx)
        {
            const auto deserialized = DeserializeFixedSizePOD<typename MatrixType::Scalar>(buffer, current + bytes_read);
            eigen.data()[idx] = deserialized.first;
            bytes_read += deserialized.second;
        }
        return {eigen, bytes_read};
    }

    inline uint64_t SerializeEigenVector3d(
            const Eigen::Vector3d& eigen,
            std::vector<uint8_t>& buffer)
    {
        return SerializeEigen(eigen, buffer);
    }

    inline uint64_t SerializeEigenVectorXd(
            const Eigen::VectorXd& eigen,
            std::vector<uint8_t>& buffer)
    {
        return SerializeEigen(eigen, buffer);
    }

    inline uint64_t SerializeEigenMatrix3d(
            const Eigen::Matrix3d& eigen,
            std::vector<uint8_t>& buffer)
    {
        return SerializeEigen(eigen, buffer);
    }

    inline uint64_t SerializeEigenMatrixXd(
            const Eigen::MatrixXd& eigen,
            std::vector<uint8_t>& buffer)
    {
        return SerializeEigen(eigen, buffer);
    }

    inline std::pair<Eigen::Vector3d, uint64_t> DeserializeEigenVector3d(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        return DeserializeFixedSizeEigen<Eigen::Vector3d>(buffer, current);
    }

    inline std::pair<Eigen::VectorXd, uint64_t> DeserializeEigenVectorXd(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const int64_t length)
    {
        return DeserializeDynamicSizeEigen<Eigen::VectorXd>(buffer, current, length, 1);
    }

    inline std::pair<Eigen::Matrix3d, uint64_t> DeserializeEigenMatrix3d(
            const std::vector<uint8_t>& buffer,
            const uint64_t current)
    {
        return DeserializeFixedSizeEigen<Eigen::Matrix3d>(buffer, current);
    }

    inline std::pair<Eigen::MatrixXd, uint64_t> DeserializeEigenMatrixXd(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const int64_t rows,
            const int64_t cols)
    {
        return DeserializeDynamicSizeEigen<Eigen::MatrixXd>(buffer, current, rows, cols);
    }
}

#endif // SERIALIZATION_EIGEN_HPP
