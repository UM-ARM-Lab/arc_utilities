#include "arc_utilities/serialization_eigen.hpp"
#include "arc_utilities/arc_helpers.hpp"

template<typename Scalar, int _Rows>
void testFloatVectors(const size_t num_tests, const ssize_t rows = -1)
{
    typedef Eigen::Matrix<Scalar, _Rows, 1> EigenType;

    for (size_t idx = 0; idx < num_tests; ++idx)
    {
        EigenType vec;
        if (_Rows == Eigen::Dynamic)
        {
            assert(rows > 0);
            vec.resize(rows);
        }

        vec.setRandom();

        // First, serialize
        std::vector<uint8_t> buffer;
        const size_t bytes_used = arc_utilities::SerializeEigen(vec, buffer);
        UNUSED(bytes_used);

        // Then deserialze and compare
        const auto deserialized = arc_utilities::DeserializeEigen<EigenType>(buffer, 0);
        UNUSED(deserialized);
        assert(deserialized.second == bytes_used);
        assert(deserialized.first == vec);
    }
}

template<typename Scalar, int _Rows, int _Cols>
void testFloatMatrices(const size_t num_tests, const ssize_t rows = -1, const ssize_t cols = -1)
{
    typedef Eigen::Matrix<Scalar, _Rows, _Cols> EigenType;

    for (size_t idx = 0; idx < num_tests; ++idx)
    {
        EigenType matrix;
        if (_Rows == Eigen::Dynamic || _Cols == Eigen::Dynamic)
        {
            assert(rows > 0 && cols > 0);
            matrix.resize(rows, cols);
        }

        matrix.setRandom();

        // First, serialize
        std::vector<uint8_t> buffer;
        const size_t bytes_used = arc_utilities::SerializeEigen(matrix, buffer);
        UNUSED(bytes_used);

        // Then deserialze and compare
        const auto deserialized = arc_utilities::DeserializeEigen<EigenType>(buffer, 0);
        UNUSED(deserialized);
        assert(deserialized.second == bytes_used);
        assert(deserialized.first == matrix);
    }
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // Note that deserialization functions are only written for some size of matrices, and mostly only for doubles
    testFloatVectors<double, 3>(10);
    testFloatVectors<double, 6>(10);
    testFloatVectors<double, 7>(10);
    testFloatVectors<double, Eigen::Dynamic>(10, 20);

    testFloatMatrices<double, Eigen::Dynamic, Eigen::Dynamic>(10, 40, 50);
    testFloatMatrices<double, 3, Eigen::Dynamic>(10, 3, 50);
    testFloatMatrices<float, 3, Eigen::Dynamic>(10, 3, 50);

    std::cout << "All tests passed" << std::endl;

    return EXIT_SUCCESS;
}
