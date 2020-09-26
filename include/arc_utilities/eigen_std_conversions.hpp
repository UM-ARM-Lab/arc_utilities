#ifndef LGV_EIGEN_STD_CONVERSIONS
#define LGV_EIGEN_STD_CONVERSIONS

#include <Eigen/Dense>
#include <vector>
#include <cstdlib>

template<typename Output>
Output ConvertTo(Eigen::VectorXd const& vec);

template<>
std::vector<double> ConvertTo<std::vector<double>>(Eigen::VectorXd const& vec)
{
    std::vector<double> std_vec(vec.size());
    std::memcpy(std_vec.data(), vec.data(), vec.size() * sizeof(double));
    return std_vec;
}

#endif // LGV_EIGEN_STD_CONVERSIONS
