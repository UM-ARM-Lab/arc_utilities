#ifndef SHORTCUT_SMOOTHING_HPP
#define SHORTCUT_SMOOTHING_HPP

#include <functional>
#include "arc_utilities/eigen_helpers.hpp"

namespace shortcut_smoothing
{
    EigenHelpers::VectorVector3d ShortcutSmooth(const EigenHelpers::VectorVector3d& input_vector, const size_t first_ind, const size_t second_ind, const double step_size, const std::function<bool(const Eigen::Vector3d&)>& collision_fn)
    {
        const size_t starting_ind = std::min(first_ind, second_ind);
        const size_t ending_ind = std::max(first_ind, second_ind);

        const Eigen::Vector3d& starting_point = input_vector[starting_ind];
        const Eigen::Vector3d& ending_point = input_vector[ending_ind];
        const Eigen::Vector3d delta = ending_point - starting_point;
        const Eigen::Vector3d delta_unit_vec = delta.normalized();

        const double total_dist = delta.norm();

        if (total_dist <= step_size)
        {
            return input_vector;
        }

        bool collision = false;
        for (double dist = step_size; !collision && dist < total_dist; dist += step_size)
        {
            const Eigen::Vector3d point_to_check = starting_point + dist * delta_unit_vec;
            collision = collision_fn(point_to_check);
        }


        if (!collision)
        {
            const size_t num_original_elements = ending_ind - starting_ind - 1;
            const size_t num_new_elements = (size_t)std::ceil(total_dist / step_size);

            EigenHelpers::VectorVector3d output_vector(input_vector.size() - num_original_elements + num_new_elements - 1);

            // Copy in the first unchanged elements of the vector
            std::copy(input_vector.begin(), input_vector.begin() + starting_ind + 1, output_vector.begin());

            // Assign the replaced elements
            for (size_t new_element_ind = 1; new_element_ind < num_new_elements; ++new_element_ind)
            {
                const double dist = (double)new_element_ind * step_size;
                output_vector[starting_ind + new_element_ind] = starting_point + dist * delta_unit_vec;
            }

            // Copy in the last unchanged elements of the vector
            std::copy(input_vector.begin() + ending_ind, input_vector.end(), output_vector.begin() + starting_ind + num_new_elements);

            return output_vector;

        }
        else
        {
            return input_vector;
        }
    }
}

#endif // SHORTCUT_SMOOTHING_HPP
