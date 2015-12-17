#include <stdlib.h>
#include <functional>
#include <Eigen/Geometry>
#include <type_traits>
#include <random>

#ifdef ENABLE_PARALLEL
#include <omp.h>
#endif

#ifndef ARC_HELPERS_HPP
#define ARC_HELPERS_HPP

// Branch prediction hints
// Figure out which compiler we have
#if defined(__clang__)
    /* Clang/LLVM */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__ICC) || defined(__INTEL_COMPILER)
    /* Intel ICC/ICPC */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(__GNUC__) || defined(__GNUG__)
    /* GNU GCC/G++ */
    #define likely(x) __builtin_expect(!!(x), 1)
    #define unlikely(x) __builtin_expect(!!(x), 0)
#elif defined(_MSC_VER)
    /* Microsoft Visual Studio */
    /* MSVC doesn't support branch prediction hints. Use PGO instead. */
    #define likely(x) (x)
    #define unlikely(x) (x)
#endif

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

namespace arc_helpers
{
    template <typename T>
    inline T SetBit(const T current, const u_int32_t bit_position, const bool bit_value)
    {
        // Safety check on the type we've been called with
        static_assert((std::is_same<T, u_int8_t>::value
                       || std::is_same<T, u_int16_t>::value
                       || std::is_same<T, u_int32_t>::value
                       || std::is_same<T, u_int64_t>::value),
                      "Type must be a fixed-size unsigned integral type");
        // Do it
        T update_mask = 1;
        update_mask = update_mask << bit_position;
        if (bit_value)
        {
            return (current | update_mask);
        }
        else
        {
            update_mask = (~update_mask);
            return (current & update_mask);
        }
    }

    template<typename Datatype, typename Allocator=std::allocator<Datatype>>
    static Eigen::MatrixXd BuildDistanceMatrix(const std::vector<Datatype, Allocator>& data, std::function<double(const Datatype&, const Datatype&)>& distance_fn)
    {
        Eigen::MatrixXd distance_matrix(data.size(), data.size());
#ifdef ENABLE_PARALLEL
        #pragma omp parallel for schedule(guided)
#endif
        for (size_t idx = 0; idx < data.size(); idx++)
        {
            for (size_t jdx = 0; jdx < data.size(); jdx++)
            {
                distance_matrix(idx, jdx) = distance_fn(data[idx], data[jdx]);
            }
        }
        return distance_matrix;
    }

    class TruncatedNormalDistribution
    {
    protected:

        double mean_;
        double stddev_;
        double std_lower_bound_;
        double std_upper_bound_;

        enum CASES {TYPE_1, TYPE_2, TYPE_3, TYPE_4, NONE};
        CASES case_;
        std::uniform_real_distribution<double> uniform_unit_dist_;
        std::uniform_real_distribution<double> uniform_range_dist_;
        std::exponential_distribution<double> exponential_dist_;
        std::normal_distribution<double> normal_dist_;

        inline bool CheckSimple(const double lower_bound, const double upper_bound) const
        {
            // Init Values Used in Inequality of Interest
            const double val1 = (2 * sqrt(exp(1))) / (lower_bound + sqrt(pow(lower_bound, 2) + 4));
            const double val2 = exp((pow(lower_bound, 2) - lower_bound * sqrt(pow(lower_bound, 2) + 4)) / (4));
            if (upper_bound > lower_bound + val1 * val2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        // Naive Accept-Reject algorithm
        template<typename Generator>
        inline double NaiveAcceptReject(const double lower_bound, const double upper_bound, Generator& prng)
        {
            while (true)
            {
                const double draw = normal_dist_(prng); // Rf_rnorm(0.0, 1.0) ; // Normal distribution (i.e. std::normal_distribution<double>)
                if ((draw <= upper_bound) && (draw >= lower_bound))
                {
                    return draw;
                }
            }
        }

        // Accept-Reject Algorithm
        template<typename Generator>
        inline double SimpleAcceptReject(const double lower_bound, Generator& prng)
        {
            // Init Values
            const double alpha = (lower_bound + sqrt(pow(lower_bound, 2) + 4.0)) / (2.0) ;
            while (true)
            {
                const double e = exponential_dist_(prng); // Rf_rexp(1.0) ; // Exponential distribution (i.e. std::exponential_distribution<double>)
                const double z = lower_bound + e / alpha;
                const double rho = exp(-pow(alpha - z, 2) / 2);
                const double u = uniform_unit_dist_(prng); //  Rf_runif(0, 1) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                if (u <= rho)
                {
                    return z;
                }
            }
        }

        // Accept-Reject Algorithm
        template<typename Generator>
        inline double ComplexAcceptReject(const double lower_bound, const double upper_bound, Generator& prng)
        {
            while (true)
            {
                const double z = uniform_range_dist_(prng); // Rf_runif(lower_bound, upper_bound) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                double rho = 0.0;
                if (0 < lower_bound)
                {
                    rho = exp((pow(lower_bound, 2) - pow(z, 2)) / 2);
                }
                else if (upper_bound < 0)
                {
                    rho = exp((pow(upper_bound, 2) - pow(z, 2)) / 2);
                }
                else if (0 < upper_bound && lower_bound < 0)
                {
                    rho = exp(- pow(z, 2) / 2);
                }
                const double u = uniform_unit_dist_(prng); // Rf_runif(0, 1) ; // Uniform distribution (i.e. std::uniform_real_distribution<double>)
                if (u <= rho)
                {
                    return z;
                }
            }
        }

        template<typename Generator>
        inline double Sample(Generator& prng)
        {
            if (case_ == TYPE_1)
            {
                const double draw = NaiveAcceptReject(std_lower_bound_, std_upper_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else if (case_ == TYPE_2)
            {
                const double draw = SimpleAcceptReject(std_lower_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else if (case_ == TYPE_3)
            {
                while (true)
                {
                    const double draw = SimpleAcceptReject(std_lower_bound_, prng); // use the simple algorithm if it is more efficient
                    if (draw <= std_upper_bound_)
                    {
                        return mean_ + stddev_ * draw;
                    }
                }
            }
            else if (case_ == TYPE_4)
            {
                const double draw = ComplexAcceptReject(std_lower_bound_, std_upper_bound_, prng);
                return mean_ + stddev_ * draw;
            }
            else
            {
                assert((case_ == TYPE_1) || (case_ == TYPE_2) || (case_ == TYPE_3) || (case_ == TYPE_4));
                return 0.0; // Make the compiler happy - THIS SHOULD BE IMPOSSIBLE!
            }
        }

    public:

        inline TruncatedNormalDistribution(const double mean, const double stddev, const double lower_bound, const double upper_bound) : uniform_unit_dist_(0.0, 1.0), uniform_range_dist_(lower_bound, upper_bound), exponential_dist_(1.0), normal_dist_(0.0, 1.0)
        {
            // Set operating parameters
            mean_ = mean;
            stddev_ = stddev;
            // Standardize the lower and upper bounds
            std_lower_bound_ = (lower_bound - mean_) / stddev_;
            std_upper_bound_ = (upper_bound - mean_) / stddev_;
            // Set the operating case - i.e. which sampling method we will use
            case_ = NONE;
            if (0 <= std_upper_bound_ && 0 >= std_lower_bound_)
            {
                case_ = TYPE_1;
            }
            if (0 < std_lower_bound_ && std_upper_bound_ == INFINITY)
            {
                case_ = TYPE_2;
            }
            if (0 > std_upper_bound_ && std_lower_bound_ == -INFINITY)
            {
                std_lower_bound_ = -1 * std_upper_bound_;
                std_upper_bound_ = INFINITY;
                stddev_ = -1 * stddev_;
                case_ = TYPE_2;
            }
            if ((0 > std_upper_bound_ || 0 < std_lower_bound_) && !(std_upper_bound_ == INFINITY || std_lower_bound_ == -INFINITY))
            {
                if (CheckSimple(std_lower_bound_, std_upper_bound_))
                {
                    case_ = TYPE_3;
                }
                else
                {
                    case_ = TYPE_4;
                }
            }
            assert((case_ == TYPE_1) || (case_ == TYPE_2) || (case_ == TYPE_3) || (case_ == TYPE_4));
        }

        template<typename Generator>
        inline double operator() (Generator& prng)
        {
            return Sample(prng);
        }
    };
}

#endif // ARC_HELPERS_HPP
