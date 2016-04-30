#include <stdlib.h>
#include <iostream>
#include <functional>
#include <Eigen/Geometry>
#include <type_traits>
#include <random>
#include <array>

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
    inline T SetBit(const T current, const uint32_t bit_position, const bool bit_value)
    {
        // Safety check on the type we've been called with
        static_assert((std::is_same<T, uint8_t>::value
                       || std::is_same<T, uint16_t>::value
                       || std::is_same<T, uint32_t>::value
                       || std::is_same<T, uint64_t>::value),
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
    static Eigen::MatrixXd BuildDistanceMatrix(const std::vector<Datatype, Allocator>& data, const std::function<double(const Datatype&, const Datatype&)>& distance_fn)
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

    class SplitMix64PRNG
    {
    private:

        uint64_t state_; /* The state can be seeded with any value. */

        inline uint64_t next(void)
        {
            uint64_t z = (state_ += UINT64_C(0x9E3779B97F4A7C15));
            z = (z ^ (z >> 30)) * UINT64_C(0xBF58476D1CE4E5B9);
            z = (z ^ (z >> 27)) * UINT64_C(0x94D049BB133111EB);
            return z ^ (z >> 31);
        }

    public:

        inline SplitMix64PRNG(const uint64_t seed_val)
        {
            seed(seed_val);
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            state_ = seed_val;
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };

    class XorShift128PlusPRNG
    {
    private:

        uint64_t state_1_;
        uint64_t state_2_;

        inline uint64_t next(void)
        {
            uint64_t s1 = state_1_;
            const uint64_t s0 = state_2_;
            state_1_ = s0;
            s1 ^= s1 << 23; // a
            state_2_ = s1 ^ s0 ^ (s1 >> 18) ^ (s0 >> 5); // b, c
            return state_2_ + s0;
        }

    public:

        inline XorShift128PlusPRNG(const uint64_t seed_val)
        {
            seed(seed_val);
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            SplitMix64PRNG temp_seed_gen(seed_val);
            state_1_ = temp_seed_gen();
            state_2_ = temp_seed_gen();
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };

    class XorShift1024StarPRNG
    {
    private:

        std::array<uint64_t, 16> state_;
        int32_t p;

        inline uint64_t next(void)
        {
            const uint64_t s0 = state_[(size_t)p];
            p = (p + 1) & 15;
            uint64_t s1 = state_[(size_t)p];
            s1 ^= s1 << 31; // a
            state_[(size_t)p] = s1 ^ s0 ^ (s1 >> 11) ^ (s0 >> 30); // b,c
            return state_[(size_t)p] * UINT64_C(1181783497276652981);
        }

    public:

        inline XorShift1024StarPRNG(const uint64_t seed_val)
        {
            seed(seed_val);
            p = 0;
        }

        static constexpr uint64_t min(void)
        {
            return 0u;
        }

        static constexpr uint64_t max(void)
        {
            return std::numeric_limits<uint64_t>::max();
        }

        inline void seed(const uint64_t seed_val)
        {
            SplitMix64PRNG temp_seed_gen(seed_val);
            for (size_t idx = 0u; idx < state_.size(); idx++)
            {
                state_[idx] = temp_seed_gen();
            }
        }

        inline void discard(const unsigned long long z)
        {
            uint64_t temp __attribute__((unused)); // This suppresses "set but not used" warnings
            temp = 0u;
            for (unsigned long long i = 0; i < z; i++)
            {
                temp = next();
                __asm__ __volatile__(""); // This should prevent the compiler from optimizing out the loop
            }
        }

        inline uint64_t operator() (void)
        {
            return next();
        }
    };

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
                assert(case_ == NONE);
                return mean_;
            }
        }

    public:

        inline TruncatedNormalDistribution(const double mean, const double stddev, const double lower_bound, const double upper_bound) : uniform_unit_dist_(0.0, 1.0), uniform_range_dist_(lower_bound, upper_bound), exponential_dist_(1.0), normal_dist_(0.0, 1.0)
        {
            // Set operating parameters
            mean_ = mean;
            stddev_ = stddev;
            if (fabs(stddev_) == 0.0)
            {
                case_ = NONE;
            }
            else
            {
                // Standardize the lower and upper bounds
                std_lower_bound_ = (lower_bound - mean_) / stddev_;
                std_upper_bound_ = (upper_bound - mean_) / stddev_;
                // Set the operating case - i.e. which sampling method we will use
                case_ = NONE;
                if (0.0 <= std_upper_bound_ && 0.0 >= std_lower_bound_)
                {
                    case_ = TYPE_1;
                }
                if (0.0 < std_lower_bound_ && std_upper_bound_ == INFINITY)
                {
                    case_ = TYPE_2;
                }
                if (0.0 > std_upper_bound_ && std_lower_bound_ == -INFINITY)
                {
                    std_lower_bound_ = -1 * std_upper_bound_;
                    std_upper_bound_ = INFINITY;
                    stddev_ = -1 * stddev_;
                    case_ = TYPE_2;
                }
                if ((0.0 > std_upper_bound_ || 0.0 < std_lower_bound_) && !(std_upper_bound_ == INFINITY || std_lower_bound_ == -INFINITY))
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
        }

        template<typename Generator>
        inline double operator() (Generator& prng)
        {
            return Sample(prng);
        }
    };

    class RandomRotationGenerator
    {
    protected:

        std::uniform_real_distribution<double> uniform_unit_dist_;

        // From: "Uniform Random Rotations", Ken Shoemake, Graphics Gems III, pg. 124-132
        template<typename Generator>
        inline Eigen::Quaterniond GenerateUniformRandomQuaternion(Generator& prng)
        {
            const double x0 = uniform_unit_dist_(prng);
            const double r1 = sqrt(1.0 - x0);
            const double r2 = sqrt(x0);
            const double t1 = 2.0 * M_PI * uniform_unit_dist_(prng);
            const double t2 = 2.0 * M_PI * uniform_unit_dist_(prng);
            const double c1 = cos(t1);
            const double s1 = sin(t1);
            const double c2 = cos(t2);
            const double s2 = sin(t2);
            const double x = s1 * r1;
            const double y = c1 * r1;
            const double z = s2 * r2;
            const double w = c2 * r2;
            return Eigen::Quaterniond(w, x, y, z);
        }

        // From Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning, by James Kuffner, ICRA 2004
        template<typename Generator>
        Eigen::Vector3d GenerateUniformRandomEulerAngles(Generator& prng)
        {
            const double roll = M_PI * (-2.0 * uniform_unit_dist_(prng) + 1.0);
            const double pitch = acos(1.0 - 2.0 * uniform_unit_dist_(prng)) - M_PI_2;
            const double yaw = M_PI * (-2.0 * uniform_unit_dist_(prng) + 1.0);
            return Eigen::Vector3d(roll, pitch, yaw);
        }

    public:

        inline RandomRotationGenerator() : uniform_unit_dist_(0.0, 1.0) {}

        template<typename Generator>
        inline Eigen::Quaterniond GetQuaternion(Generator& prng)
        {
            return GenerateUniformRandomQuaternion(prng);
        }

        template<typename Generator>
        inline std::vector<double> GetRawQuaternion(Generator& prng)
        {
            const Eigen::Quaterniond quat = GenerateUniformRandomQuaternion(prng);
            return std::vector<double>{quat.x(), quat.y(), quat.z(), quat.w()};
        }

        template<typename Generator>
        inline Eigen::Vector3d GetEulerAngles(Generator& prng)
        {
            return GenerateUniformRandomEulerAngles(prng);
        }

        template<typename Generator>
        inline std::vector<double> GetRawEulerAngles(Generator& prng)
        {
            const Eigen::Vector3d angles = GenerateUniformRandomEulerAngles(prng);
            return std::vector<double>{angles.x(), angles.y(), angles.z()};
        }
    };

    template<typename T>
    inline uint64_t SerializeFixedSizePOD(const T& item_to_serialize, std::vector<uint8_t>& buffer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // Fixed-size serialization via memcpy
        std::vector<uint8_t> temp_buffer(sizeof(item_to_serialize), 0x00);
        memcpy(&temp_buffer[0], &item_to_serialize, sizeof(item_to_serialize));
        // Move to buffer
        buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T>
    inline std::pair<T, uint64_t> DeserializeFixedSizePOD(const std::vector<uint8_t>& buffer, const uint64_t current)
    {
        T temp_item;
        assert(current <= buffer.size());
        assert((current + sizeof(temp_item)) <= buffer.size());
        memcpy(&temp_item, &buffer[current], sizeof(temp_item));
        return std::make_pair(temp_item, sizeof(temp_item));
    }

    template<typename T, typename Allocator=std::allocator<T>>
    inline uint64_t SerializeVector(const std::vector<T, Allocator>& vec_to_serialize, std::vector<uint8_t>& buffer, const std::function<uint64_t(const T&, std::vector<uint8_t>&)>& item_serializer)
    {
        const uint64_t start_buffer_size = buffer.size();
        // First, write a uint64_t size header
        const uint64_t size = (uint64_t)vec_to_serialize.size();
        std::vector<uint8_t> size_header(sizeof(size), 0x00);
        memcpy(&size_header[0], &size, sizeof(size));
        // Move to buffer
        buffer.insert(buffer.end(), size_header.begin(), size_header.end());
        // Serialize the contained items
        for (size_t idx = 0; idx < size; idx++)
        {
            const T& current = vec_to_serialize[idx];
            item_serializer(current, buffer);
        }
        // Figure out how many bytes were written
        const uint64_t end_buffer_size = buffer.size();
        const uint64_t bytes_written = end_buffer_size - start_buffer_size;
        return bytes_written;
    }

    template<typename T, typename Allocator=std::allocator<T>>
    inline std::pair<std::vector<T, Allocator>, uint64_t> DeserializeVector(const std::vector<uint8_t>& buffer, const uint64_t current, const std::function<std::pair<T, uint64_t>(const std::vector<uint8_t>&, const uint64_t)>& item_deserializer)
    {
        // First, try to load the header
        assert(current < buffer.size());
        assert((current + sizeof(uint64_t)) <= buffer.size());
        // Load the header
        uint64_t size = 0u;
        memcpy(&size, &buffer[current], sizeof(uint64_t));
        // Deserialize the items
        std::vector<T, Allocator> deserialized;
        deserialized.reserve(size);
        uint64_t current_position = current + sizeof(uint64_t);
        for (uint64_t idx = 0; idx < size; idx++)
        {
            const std::pair<T, uint64_t> deserialized_item = item_deserializer(buffer, current_position);
            deserialized.push_back(deserialized_item.first);
            current_position += deserialized_item.second;
        }
        // Figure out how many bytes were read
        const uint64_t bytes_read = current_position - current;
        return std::make_pair(deserialized, bytes_read);
    }
}

#endif // ARC_HELPERS_HPP
