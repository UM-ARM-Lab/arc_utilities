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

#ifndef SIMPLE_HAUSDORFF_DISTANCE_HPP
#define SIMPLE_HAUSDORFF_DISTANCE_HPP

namespace simple_hausdorff_distance
{
    class SimpleHausdorffDistance
    {
    private:

        SimpleHausdorffDistance() {}

    public:

        template<typename Datatype, typename Allocator=std::allocator<Datatype>>
        static double ComputeDistance(const std::vector<Datatype, Allocator>& first_distribution, const std::vector<Datatype, Allocator>& second_distribution, const std::function<double(const Datatype&, const Datatype&)>& distance_fn)
        {
            // Compute the Hausdorff distance - the "maximum minimum" distance
            double maximum_minimum_distance = 0.0;
#ifdef ENABLE_PARALLEL
            std::mutex dist_mutex;
            #pragma omp parallel for schedule(guided)
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
#ifdef ENABLE_PARALLEL
                std::lock_guard<std::mutex> lock(dist_mutex);
#endif
                if (minimum_distance > maximum_minimum_distance)
                {
                    maximum_minimum_distance = minimum_distance;
                }
            }
            return maximum_minimum_distance;
        }
    };
}
#endif // SIMPLE_HAUSDORFF_DISTANCE_HPP
