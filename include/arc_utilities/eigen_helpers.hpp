#include <Eigen/StdVector>
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
        if ((v1.x() == v2.x()) && (v1.y() == v2.y()) && (v1.z() == v2.z()))
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
    // Serialization/Deserialization functions
    ////////////////////////////////////////////////////////////////////////////

    // Prototypes for serialization/deserialization functions
    template<typename Container>
    inline uint64_t SerializedSize(const Container& value);

    // For fixed-size containers only (others have a uint64_t size header first)
    template<typename Container>
    inline uint64_t SerializedSize(void);

    template<typename Container>
    inline uint64_t Serialize(const Container& value, std::vector<uint8_t>& buffer);

    template<typename Container>
    inline std::pair<Container, uint64_t> Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current);

    // Concrete implementations
    template<>
    inline uint64_t SerializedSize(const Eigen::VectorXd& value)
    {
        (void)(value);
        return (uint64_t)((1 * sizeof(uint64_t)) + (value.size() * sizeof(double))); // Space for a uint64_t size header and the data
    }

    template<>
    inline uint64_t Serialize(const Eigen::VectorXd& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        const uint64_t serialized_size = SerializedSize(value);
        std::vector<uint8_t> temp_buffer(serialized_size, 0x00);
        // Make the header
        const uint64_t size_header = (uint64_t)value.size();
        memcpy(&temp_buffer.front(), & size_header, sizeof(size_header));
        // Copy the data
        memcpy(&(temp_buffer[sizeof(size_header)]), value.data(), (serialized_size - sizeof(size_header)));
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return serialized_size;
    }

    template<>
    inline std::pair<Eigen::VectorXd, uint64_t> Deserialize<Eigen::VectorXd>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + sizeof(uint64_t)) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        // Load the header
        uint64_t size_header = 0u;
        memcpy(&size_header, &buffer[current], sizeof(uint64_t));
        // Check buffer size
        Eigen::VectorXd temp_value = Eigen::VectorXd::Zero(size_header);
        const uint64_t serialized_size = SerializedSize(temp_value);
        assert((current + serialized_size) <= buffer.size());
        // Load from the buffer
        memcpy(temp_value.data(), &buffer[current + sizeof(size_header)], (serialized_size - sizeof(size_header)));
        return std::make_pair(temp_value, serialized_size);
    }

    template<>
    inline uint64_t SerializedSize(const Eigen::Vector3d& value)
    {
        (void)(value);
        return (uint64_t)(3 * sizeof(double));
    }

    template<>
    inline uint64_t SerializedSize<Eigen::Vector3d>(void)
    {
        return (uint64_t)(3 * sizeof(double));
    }

    template<>
    inline uint64_t Serialize(const Eigen::Vector3d& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        std::vector<uint8_t> temp_buffer(SerializedSize<Eigen::Vector3d>(), 0x00);
        memcpy(&temp_buffer.front(), value.data(), SerializedSize<Eigen::Vector3d>());
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return SerializedSize<Eigen::Vector3d>();
    }

    template<>
    inline std::pair<Eigen::Vector3d, uint64_t> Deserialize<Eigen::Vector3d>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + SerializedSize<Eigen::Vector3d>()) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        Eigen::Vector3d temp_value;
        memcpy(temp_value.data(), &buffer[current], SerializedSize<Eigen::Vector3d>());
        return std::make_pair(temp_value, SerializedSize<Eigen::Vector3d>());
    }

    template<>
    inline uint64_t SerializedSize(const Eigen::Matrix<double, 6, 1>& value)
    {
        (void)(value);
        return (uint64_t)(6 * sizeof(double));
    }

    template<>
    inline uint64_t SerializedSize<Eigen::Matrix<double, 6, 1>>(void)
    {
        return (uint64_t)(6 * sizeof(double));
    }

    template<>
    inline uint64_t Serialize(const Eigen::Matrix<double, 6, 1>& value, std::vector<uint8_t>& buffer)
    {
        // Takes a state to serialize and a buffer to serialize into
        // Return number of bytes written to buffer
        std::vector<uint8_t> temp_buffer(SerializedSize<Eigen::Matrix<double, 6, 1>>(), 0x00);
        memcpy(&temp_buffer.front(), value.data(), SerializedSize<Eigen::Matrix<double, 6, 1>>());
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        return SerializedSize<Eigen::Matrix<double, 6, 1>>();
    }

    template<>
    inline std::pair<Eigen::Matrix<double, 6, 1>, uint64_t> Deserialize<Eigen::Matrix<double, 6, 1>>(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        assert(current < buffer.size());
        assert((current + SerializedSize<Eigen::Matrix<double, 6, 1>>()) <= buffer.size());
        // Takes a buffer to read from and the starting index in the buffer
        // Return the loaded state and how many bytes we read from the buffer
        Eigen::Matrix<double, 6, 1> temp_value;
        memcpy(temp_value.data(), &buffer[current], SerializedSize<Eigen::Matrix<double, 6, 1>>());
        return std::make_pair(temp_value, SerializedSize<Eigen::Matrix<double, 6, 1>>());
    }

    ////////////////////////////////////////////////////////////////////////////
    // Helper functions
    ////////////////////////////////////////////////////////////////////////////

    inline double EnforceContinuousRevoluteBounds(const double value)
    {
        if ((value <= -M_PI) || (value > M_PI))
        {
            const double remainder = fmod(value, 2.0 * M_PI);
            if (remainder <= -M_PI)
            {
                return (remainder + (2.0 * M_PI));
            }
            else if (remainder > M_PI)
            {
                return (remainder - (2.0 * M_PI));
            }
            else
            {
                return remainder;
            }
        }
        else
        {
            return value;
        }
    }

    inline Eigen::Vector3d SafeNorm(const Eigen::Vector3d& vec)
    {
        const double norm = vec.norm();
        if (norm > std::numeric_limits<double>::epsilon())
        {
            return vec / norm;
        }
        else
        {
            return vec;
        }
    }

    inline Eigen::Matrix3d Skew(const Eigen::Vector3d& vector)
    {
        Eigen::Matrix3d skewed;
        skewed << 0.0, -vector.z(), vector.y(),
                  vector.z(), 0.0, -vector.x(),
                  -vector.y(), vector.x(), 0.0;
        return skewed;
    }

    inline Eigen::Vector3d Unskew(const Eigen::Matrix3d& matrix)
    {
        const Eigen::Matrix3d matrix_symetric = (matrix - matrix.transpose()) / 2.0;
        const Eigen::Vector3d unskewed(matrix_symetric(2, 1), matrix_symetric(0, 2), matrix_symetric(1, 0));
        return unskewed;
    }

    inline Eigen::Matrix<double, 6, 6> AdjointFromTransform(const Eigen::Affine3d& transform)
    {
        const Eigen::Matrix3d rotation = transform.matrix().block<3, 3>(0, 0);
        const Eigen::Vector3d translation = transform.matrix().block<3, 1>(0, 3);
        const Eigen::Matrix3d translation_hat = Skew(translation);
        // Assemble the adjoint matrix
        Eigen::Matrix<double, 6, 6> adjoint;
        adjoint.block<3, 3>(0, 0) = rotation;
        adjoint.block<3, 3>(0, 3) = translation_hat * rotation;
        adjoint.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
        adjoint.block<3, 3>(3, 3) = rotation;
        return adjoint;
    }

    inline Eigen::Matrix<double, 6, 1> TransformTwist(const Eigen::Affine3d& transform, const Eigen::Matrix<double, 6, 1>& initial_twist)
    {
        return (Eigen::Matrix<double, 6, 1>)(EigenHelpers::AdjointFromTransform(transform) * initial_twist);
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

    inline double InterpolateContinuousRevolute(const double p1, const double p2, const double ratio)
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
        // Safety check args
        const double real_p1 = EnforceContinuousRevoluteBounds(p1);
        const double real_p2 = EnforceContinuousRevoluteBounds(p2);
        // Interpolate
        double interpolated = 0.0;
        double diff = real_p2 - real_p1;
        if (fabs(diff) <= M_PI)
        {
            interpolated = real_p1 + diff * ratio;
        }
        else
        {
            if (diff > 0.0)
            {
                diff = 2.0 * M_PI - diff;
            }
            else
            {
                diff = -2.0 * M_PI - diff;
            }
            interpolated = real_p1 - diff * ratio;
            // Input states are within bounds, so the following check is sufficient
            if (interpolated > M_PI)
            {
                interpolated -= 2.0 * M_PI;
            }
            else
            {
                if (interpolated < -M_PI)
                {
                    interpolated += 2.0 * M_PI;
                }
            }
        }
        return interpolated;
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
        const Eigen::Vector3d v1 = t1.translation();
        const Eigen::Quaterniond q1(t1.rotation());
        const Eigen::Vector3d v2 = t2.translation();
        const Eigen::Quaterniond q2(t2.rotation());
        const Eigen::Vector3d vint = Interpolate(v1, v2, real_ratio);
        const Eigen::Quaterniond qint = Interpolate(q1, q2, real_ratio);
        const Eigen::Affine3d tint = ((Eigen::Translation3d)vint) * qint;
        return tint;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Distance functions
    ////////////////////////////////////////////////////////////////////////////

    inline double Distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        return (v2 - v1).norm();
    }

    inline double Distance(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2)
    {
        assert(v1.size() == v2.size());
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

    inline double Distance(const Eigen::Affine3d& t1, const Eigen::Affine3d& t2, const double alpha=0.5)
    {
        assert(alpha >= 0.0);
        assert(alpha <= 1.0);
        const Eigen::Vector3d v1 = t1.translation();
        const Eigen::Quaterniond q1(t1.rotation());
        const Eigen::Vector3d v2 = t2.translation();
        const Eigen::Quaterniond q2(t2.rotation());
        const double vdist = Distance(v1, v2) * (1.0 - alpha);
        const double qdist = Distance(q1, q2) * (alpha);
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

    inline double ContinuousRevoluteSignedDistance(const double p1, const double p2)
    {
        // Safety check args
        const double real_p1 = EnforceContinuousRevoluteBounds(p1);
        const double real_p2 = EnforceContinuousRevoluteBounds(p2);
        const double raw_distance = real_p2 - real_p1;
        if ((raw_distance <= -M_PI) || (raw_distance > M_PI))
        {
            if (raw_distance <= -M_PI)
            {
                return (-(2.0 * M_PI) - raw_distance);
            }
            else if (raw_distance > M_PI)
            {
                return ((2.0 * M_PI) - raw_distance);
            }
            else
            {
                return raw_distance;
            }
        }
        else
        {
            return raw_distance;
        }
    }

    inline double ContinuousRevoluteDistance(const double p1, const double p2)
    {
        return fabs(ContinuousRevoluteSignedDistance(p1, p2));
    }

    inline double AddContinuousRevoluteValues(const double start, const double change)
    {
        return EnforceContinuousRevoluteBounds(start + change);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Conversion functions
    ////////////////////////////////////////////////////////////////////////////

    inline Eigen::Vector3d EulerAnglesFromRotationMatrix(const Eigen::Matrix<double, 3, 3>& rot_matrix)
    {
        const Eigen::Vector3d euler_angles = rot_matrix.eulerAngles(0, 1, 2); // Use XYZ angles
        return euler_angles;
    }

    inline Eigen::Vector3d EulerAnglesFromQuaternion(const Eigen::Quaterniond& quat)
    {
        return EulerAnglesFromRotationMatrix(quat.toRotationMatrix());
    }

    inline Eigen::Vector3d EulerAnglesFromAffine3d(const Eigen::Affine3d& trans)
    {
        return EulerAnglesFromRotationMatrix(trans.rotation());
    }

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

    inline double AverageStdVectorDouble(const std::vector<double>& values, const std::vector<double>& weights=std::vector<double>())
    {
        assert(values.size() > 0);
        assert((weights.size() == values.size()) || (weights.size() == 0));
        if (values.size() == 1)
        {
            return values[0];
        }
        // Get the weights
        bool use_weights = false;
        double sum_weights = 0.0;
        if (weights.size() == values.size())
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
            sum_weights = (double)values.size();
        }
        // Do weighted averaging
        double average = 0.0;
        for (size_t idx = 0; idx < values.size(); idx++)
        {
            double ew = 1.0;
            if (use_weights)
            {
                ew = fabs(weights[idx]);
            }
            const double current = values[idx];
            average += (ew / sum_weights) * current;
        }
        return average;
    }

    inline double AverageContinuousRevolute(const std::vector<double>& angles, const std::vector<double>& weights=std::vector<double>())
    {
        assert(angles.size() > 0);
        assert((weights.size() == angles.size()) || (weights.size() == 0));
        if (angles.size() == 1)
        {
            return angles[0];
        }
        // Get the weights
        bool use_weights = false;
        double sum_weights = 0.0;
        if (weights.size() == angles.size())
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
            sum_weights = (double)angles.size();
        }
        // Do weighted averaging
        double average = angles[0];
        for (size_t idx = 0; idx < angles.size(); idx++)
        {
            double ew = 1.0;
            if (use_weights)
            {
                ew = fabs(weights[idx]);
            }
            const double current = angles[idx] - angles[0];
            average += (ew / sum_weights) * current;
        }
        return average;
    }

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
            const Eigen::Vector3d& current = vectors[idx];
            sum_vector += (ew / sum_weights) * current;
        }
        return sum_vector;
    }

    inline Eigen::VectorXd AverageEigenVectorXd(const std::vector<Eigen::VectorXd>& vectors, const std::vector<double>& weights=std::vector<double>())
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
        Eigen::VectorXd sum_vector = Eigen::VectorXd::Zero(vectors[0].size());
        for (size_t idx = 0; idx < vectors.size(); idx++)
        {
            double ew = 1.0;
            if (use_weights)
            {
                ew = fabs(weights[idx]);
            }
            const Eigen::VectorXd& prev_sum = sum_vector;
            const Eigen::VectorXd& current = vectors[idx];
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
            q_matrix.col((long)idx) << ew * q.w(), ew * q.x(), ew * q.y(), ew * q.z();
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
            const double current_eigenvalue = eigen_values((long)idx).real();
            if (current_eigenvalue > max_eigenvalue)
            {
                max_eigenvalue = current_eigenvalue;
                max_eigenvector_index = (int64_t)idx;
            }
        }
        assert(max_eigenvector_index >= 0);
        // Note that these are already normalized!
        const Eigen::Vector4cd best_eigenvector = eigen_vectors.col((long)max_eigenvector_index);
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
    inline Eigen::MatrixXd Pinv(const Eigen::MatrixXd& b, const double rcond, const bool enable_flip=true)
    {
        bool flip = false;
        Eigen::MatrixXd a;
        if (enable_flip && (b.rows() < b.cols()))
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
        const Eigen::MatrixXd mAdjointU = svdA.matrixU().adjoint().block(0, 0, vSingular.rows(), svdA.matrixU().adjoint().cols());
        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wconversion"
        // Pseudo-Inversion : V * S * U'
        const Eigen::MatrixXd a_pinv = (svdA.matrixV() * vPseudoInvertedSingular.asDiagonal()) * mAdjointU;
        #pragma GCC diagnostic pop
        // Flip back if need be
        if (flip)
        {
            return a_pinv.transpose();
        }
        else
        {
            return a_pinv;
        }
    }

    /**
     * @brief WeightedLeastSquaresSolver Solves the minimization problem min || Ax - b ||^2 for x, using weights w in the norm
     *                                   If the problem is ill-conditioned, adds in a damping factor. This is equivalent to
     *                                   solving A^T * diag(W) * A * x = A^T * diag(W) * b for x.
     * @param A size M x N with M > N
     * @param b size M x 1
     * @param w size M x 1
     * @param damping_threshold The smallest singular value we allow in A^T * W * A before we apply damping
     * @param damping_value The damping value we apply to the main diagonal of A^T * W * A if we exceed the threshold
     * @return size N x 1
     */
    inline Eigen::VectorXd WeightedLeastSquaresSolver(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::VectorXd& w, const double damping_threshold, const double damping_value)
    {
        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wconversion"
        Eigen::MatrixXd left_side = A.transpose() * w.asDiagonal() * A;
        #pragma GCC diagnostic pop
        const double minimum_singular_value = left_side.jacobiSvd().singularValues().minCoeff();

        if (minimum_singular_value < damping_threshold)
        {
            left_side += damping_value * Eigen::MatrixXd::Identity(left_side.rows(), left_side.cols());
        }

        // With the damping we can assume that the left side is positive definite, so use LLT to solve this
        return left_side.llt().solve(A.transpose() * w.cwiseProduct(b));
    }
}

#endif // EIGEN_HELPERS_HPP
