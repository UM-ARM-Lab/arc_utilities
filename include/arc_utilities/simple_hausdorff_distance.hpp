#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <mutex>
#include <Eigen/Geometry>

#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
    #include <omp.h>
#endif

#ifndef SIMPLE_HAUSDORFF_DISTANCE_HPP
#define SIMPLE_HAUSDORFF_DISTANCE_HPP

namespace simple_hausdorff_distance
{
    class SimpleHausdorffDistance
    {
    private:

        SimpleHausdorffDistance() {}

        static inline size_t GetNumOMPThreads()
        {
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
            size_t num_threads = 0;
            #pragma omp parallel
            {
                num_threads = (size_t)omp_get_num_threads();
            }
            return num_threads;
#else
            return 1;
#endif
        }

    public:

        template<typename Datatype, typename Allocator=std::allocator<Datatype>>
        static double ComputeDistance(const std::vector<Datatype, Allocator>& first_distribution, const std::vector<Datatype, Allocator>& second_distribution, const std::function<double(const Datatype&, const Datatype&)>& distance_fn)
        {
            // Compute the Hausdorff distance - the "maximum minimum" distance
            double maximum_minimum_distance = 0.0;
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
            std::vector<double> thread_temp_storage(GetNumOMPThreads(), 0.0);
            #pragma omp parallel for
#endif
            for (size_t idx = 0; idx < first_distribution.size(); idx++)
            {
                const Datatype& first = first_distribution[idx];
                double minimum_distance = INFINITY;
                for (size_t jdx = 0; jdx < second_distribution.size(); jdx++)
                {
                    const Datatype& second = second_distribution[jdx];
                    const double& current_distance = distance_fn(first, second);
                    if (current_distance < minimum_distance)
                    {
                        minimum_distance = current_distance;
                    }
                }
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
                const size_t current_thread_id = (size_t)omp_get_thread_num();
                if (minimum_distance > thread_temp_storage[current_thread_id])
                {
                    thread_temp_storage[current_thread_id] = minimum_distance;
                }
#else
                if (minimum_distance > maximum_minimum_distance)
                {
                    maximum_minimum_distance = minimum_distance;
                }
#endif
            }
#ifdef ENABLE_PARALLEL_HAUSDORFF_DISTANCE
            for (size_t idx = 0; idx < thread_temp_storage.size(); idx++)
            {
                const double& temp_minimum_distance = thread_temp_storage[idx];
                if (temp_minimum_distance > maximum_minimum_distance)
                {
                    maximum_minimum_distance = temp_minimum_distance;
                }
            }
#endif
            return maximum_minimum_distance;
        }
    };
}
#endif // SIMPLE_HAUSDORFF_DISTANCE_HPP
