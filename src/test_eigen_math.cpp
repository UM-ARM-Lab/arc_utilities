#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/aligned_eigen_types.hpp>

inline void TestVector3d(const size_t iterations, const Eigen::Affine3d& base_transform)
{
    Eigen::Vector3d vector3d_result(0.0, 0.0, 0.0);
    std::chrono::time_point<std::chrono::high_resolution_clock> vector3_start_time = std::chrono::high_resolution_clock::now();
    for (size_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector3d test_vector(1.0, 2.0, 3.0);
        test_vector = test_vector + vector3d_result;
        vector3d_result = base_transform * test_vector;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> vector3_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> vector3_test_time(vector3_end_time - vector3_start_time);
    std::cout << "Affine3d * Vector3d test - " << vector3_test_time.count() << "s for " << iterations << " iterations to produce " << vector3d_result << std::endl;
}

inline void TestVector4d(const size_t iterations, const Eigen::Affine3d& base_transform)
{
    Eigen::Vector4d vector4d_result(0.0, 0.0, 0.0, 1.0);
    std::chrono::time_point<std::chrono::high_resolution_clock> vector4_start_time = std::chrono::high_resolution_clock::now();
    for (size_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector4d test_vector(1.0, 2.0, 3.0, 1.0);
        test_vector = test_vector + vector4d_result;
        test_vector(3) = 1.0;
        vector4d_result = base_transform * test_vector;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> vector4_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> vector4_test_time(vector4_end_time - vector4_start_time);
    std::cout << "Affine3d * Vector4d test - " << vector4_test_time.count() << "s for " << iterations << " iterations to produce " << vector4d_result << std::endl;
}

inline void TestAlignedVector3d(const size_t iterations, const Eigen::Affine3d& base_transform)
{
    Eigen::Aligned4Vector3<double> alignedvector3d_result(0.0, 0.0, 0.0);
    std::chrono::time_point<std::chrono::high_resolution_clock> alignedvector3_start_time = std::chrono::high_resolution_clock::now();
    for (size_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Aligned4Vector3<double> test_vector(1.0, 2.0, 3.0);
        test_vector = test_vector + alignedvector3d_result;
        alignedvector3d_result = Eigen::Aligned4Vector3<double>(base_transform * test_vector);
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> alignedvector3_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> alignedvector3_test_time(alignedvector3_end_time - alignedvector3_start_time);
    std::cout << "Affine3d * AlignedVector3d test - " << alignedvector3_test_time.count() << "s for " << iterations << " iterations to produce " << alignedvector3d_result << std::endl;
}

inline void TestManual(const size_t iterations, const Eigen::Affine3d& base_transform)
{
    const Eigen::Matrix4d& base_transform_matrix = base_transform.matrix();
    Eigen::Vector4d manual_result(0.0, 0.0, 0.0, 1.0);
    std::chrono::time_point<std::chrono::high_resolution_clock> manual_start_time = std::chrono::high_resolution_clock::now();
    for (size_t idx = 0; idx < iterations; idx++)
    {
        Eigen::Vector4d test_vector(1.0, 2.0, 3.0, 1.0);
        test_vector = test_vector + manual_result;
        test_vector(3) = 1.0;
        manual_result = base_transform_matrix * test_vector;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> manual_end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> manual_test_time(manual_end_time - manual_start_time);
    std::cout << "Matrix4d * Vector4d test - " << manual_test_time.count() << "s for " << iterations << " iterations to produce " << manual_result << std::endl;
}


int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    const size_t iterations = 1000000000;
    const Eigen::Affine3d base_transform = Eigen::Translation3d(10.0, 10.0, 10.0) * Eigen::Quaterniond::Identity();
    TestVector3d(iterations, base_transform);
    TestVector3d(iterations, base_transform);
    TestVector3d(iterations, base_transform);
    TestVector3d(iterations, base_transform);
    TestVector3d(iterations, base_transform);
    //
    TestVector4d(iterations, base_transform);
    TestVector4d(iterations, base_transform);
    TestVector4d(iterations, base_transform);
    TestVector4d(iterations, base_transform);
    TestVector4d(iterations, base_transform);
    //
    TestAlignedVector3d(iterations, base_transform);
    TestAlignedVector3d(iterations, base_transform);
    TestAlignedVector3d(iterations, base_transform);
    TestAlignedVector3d(iterations, base_transform);
    TestAlignedVector3d(iterations, base_transform);
    //
    TestManual(iterations, base_transform);
    TestManual(iterations, base_transform);
    TestManual(iterations, base_transform);
    TestManual(iterations, base_transform);
    TestManual(iterations, base_transform);
    return 0;
}
