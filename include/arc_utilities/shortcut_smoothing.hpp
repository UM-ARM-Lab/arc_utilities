#include <functional>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>

#ifndef SHORTCUT_SMOOTHING_HPP
#define SHORTCUT_SMOOTHING_HPP

namespace shortcut_smoothing
{    
    inline EigenHelpers::VectorVector3d InterpolateWithCollisionCheck(
            const EigenHelpers::VectorVector3d& input_vector,
            const size_t first_ind,
            const size_t second_ind,
            const double step_size,
            const std::function<bool(const Eigen::Vector3d&)>& collision_fn)
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

        // Collision check the path between the first and second index
        // We assume that the endpoints are not in collision, so we don't check dist == 0 or dist == total_dist
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


    /**
     * @brief ShortcutSmoothPath
     * @param path
     * @param max_iterations
     * @param max_failed_iterations
     * @param max_shortcut_fraction
     * @param edge_validity_check_fn - Must match the following prototype: std::function<bool(const Configuration&, const Configuration&)>&>
     * @param prng
     * @return
     */
    template<typename PRNG, typename Configuration, typename ConfigAlloc = std::allocator<Configuration>, class EdgeValidityCheckFn>
    inline std::vector<Configuration, ConfigAlloc> ShortcutSmoothPath(
            const std::vector<Configuration, ConfigAlloc>& path,
            const uint32_t max_iterations, const uint32_t max_failed_iterations,
            const double max_shortcut_fraction,
            const EdgeValidityCheckFn& edge_validity_check_fn,
            PRNG& prng)
    {
        std::vector<Configuration, ConfigAlloc> current_path = path;
        uint32_t num_iterations = 0;
        uint32_t failed_iterations = 0;
        while (num_iterations < max_iterations && failed_iterations < max_failed_iterations && current_path.size() > 2)
        {
            // Attempt a shortcut
            const int64_t base_index = std::uniform_int_distribution<size_t>(0, current_path.size() - 1)(prng);
            // Pick an offset fraction
            const double offset_fraction = std::uniform_real_distribution<double>(-max_shortcut_fraction, max_shortcut_fraction)(prng);
            // Compute the offset index
            const int64_t offset_index = base_index + (int64_t)((double)current_path.size() * offset_fraction); // Could be out of bounds
            const int64_t safe_offset_index = arc_helpers::ClampValue(offset_index, (int64_t)0, (int64_t)(current_path.size() - 1));
            // Get start & end indices
            const size_t start_index = (size_t)std::min(base_index, safe_offset_index);
            const size_t end_index = (size_t)std::max(base_index, safe_offset_index);
            if (end_index <= start_index + 1)
            {
                num_iterations++;
                continue;
            }
            // Check if the edge is valid
            const Configuration& start_config = current_path[start_index];
            const Configuration& end_config = current_path[end_index];
            const bool edge_valid = edge_validity_check_fn(start_config, end_config);
            if (edge_valid)
            {
                // Make the shortened path
                std::vector<Configuration, ConfigAlloc> shortened_path;
                shortened_path.insert(shortened_path.end(), current_path.begin(), current_path.begin() + start_index + 1);
                shortened_path.insert(shortened_path.end(), current_path.begin() + end_index, current_path.end());
                current_path = shortened_path;
                num_iterations++;
            }
            else
            {
                num_iterations++;
                failed_iterations++;
            }
        }
        return current_path;
    }

    /**
     * @brief ResamplePathPartial Returns the resampled portion of the path between [start_ind, end_ind); *not* the whole path
     * @param path
     * @param start_ind
     * @param end_ind
     * @param resampled_state_distance
     * @param state_distance_fn - must match the following prototype: std::function<double(const Configuration&, const Configuration&, const)>
     * @param state_interpolation_fn - must match the following prototype: std::function<Configuration(const Configuration&, const Configuration&, const double)>
     * @return
     */
    template<typename Configuration, typename ConfigAlloc = std::allocator<Configuration>, class DistanceFn, class InterpolationFn>
    inline std::vector<Configuration, ConfigAlloc> ResamplePathPartial(
            const std::vector<Configuration, ConfigAlloc>& path,
            const size_t start_ind,
            const size_t end_ind,
            const double resampled_state_distance,
            const DistanceFn& state_distance_fn,
            const InterpolationFn& state_interpolation_fn)
    {
        assert(end_ind > start_ind);
        assert(end_ind <= path.size());

        // If we only have one element, to resample between, return it
        if (end_ind - start_ind == 1)
        {
            return std::vector<Configuration, ConfigAlloc>(1, path[start_ind]);
        }

        // Add the first state
        std::vector<Configuration, ConfigAlloc> resampled_path;
        resampled_path.push_back(path[start_ind]);

        // Loop through the path, adding interpolated states as needed
        for (size_t idx = start_ind; idx < end_ind - 1; idx++)
        {
            // Get the states from the original path
            const Configuration& current_state = path[idx];
            const Configuration& next_state = path[idx + 1];

            // We want to add all the intermediate states to our returned path
            const double distance = state_distance_fn(current_state, next_state);
            const double raw_num_intervals = distance / resampled_state_distance;
            const uint32_t num_segments = (uint32_t)std::ceil(raw_num_intervals);

            if (num_segments == 0u)
            {
                // Do nothing because this means distance was exactly 0, so we are going to discard the extra point
            }
            // If there's only one segment, we just add the end state of the window
            else if (num_segments == 1u)
            {
                resampled_path.push_back(next_state);
            }
            // If there is more than one segment, interpolate between previous_state and current_state (including the next_state)
            else
            {
                for (uint32_t segment = 1u; segment <= num_segments; segment++)
                {
                    const double interpolation_ratio = (double)segment / (double)num_segments;
                    const Configuration interpolated_state = state_interpolation_fn(current_state, next_state, interpolation_ratio);
                    resampled_path.push_back(interpolated_state);
                }
            }
        }
        return resampled_path;
    }

    template<typename Configuration, typename ConfigAlloc = std::allocator<Configuration>, class DistanceFn, class InterpolationFn>
    inline std::vector<Configuration, ConfigAlloc> ResamplePath(
            const std::vector<Configuration, ConfigAlloc>& path,
            const double resampled_state_distance,
            const DistanceFn& state_distance_fn,
            const InterpolationFn& state_interpolation_fn)
    {
        return ResamplePathPartial(path, 0, path.size(), resampled_state_distance, state_distance_fn, state_interpolation_fn);
    }

    /**
     * @brief UpsamplePathPartial Returns the upsampled portion of the path between [start_ind, end_ind); *not* the whole path
     * @param path
     * @param start_ind
     * @param end_ind
     * @param num_points - this is the number of points to use, including the start point and the end point - i.e. path[start_ind] and path[end_ind-1]
     * @param state_distance_fn - must match the following prototype: std::function<double(const Configuration&, const Configuration&)>
     * @param state_interpolation_fn - must match the following prototype: std::function<Configuration(const Configuration&, const Configuration&, const double)>
     * @return
     */
    template<typename Configuration, typename ConfigAlloc = std::allocator<Configuration>, class DistanceFn, class InterpolationFn>
    inline std::vector<Configuration, ConfigAlloc> UpsamplePathPartial(
            const std::vector<Configuration, ConfigAlloc>& path,
            const ssize_t start_ind,
            const ssize_t end_ind,
            const ssize_t num_points,
            const DistanceFn& state_distance_fn,
            const InterpolationFn& state_interpolation_fn)
    {
        assert(end_ind > start_ind);
        assert(end_ind <= path.size());

        // Abort if the number of points is already sufficient
        if (num_points <= end_ind - start_ind)
        {
            std::cerr << "[ResamplePathPartialNumPoints] Number of points in existing path is already larger than the desired number of points" << std::endl;
            return path;
        }

        const ssize_t num_segments = end_ind - start_ind - 1;
        assert(num_segments > 0);

        Eigen::VectorXd individual_lengths(num_segments);
        double total_dist = 0.0;
        for (ssize_t ind = start_ind; ind < end_ind - 1; ++ind)
        {
            const double dist = state_distance_fn(path[ind], path[ind + 1]);
            individual_lengths[ind - start_ind] = dist;
            total_dist += dist;
        }

        const double nominal_delta = total_dist / (num_points - 1);
        Eigen::VectorXi num_points_per_segment(num_segments);
        Eigen::VectorXd distance_consumed_per_segment(num_segments);
        for (size_t ind = start_ind; ind < end_ind - 1; ++ind)
        {
            num_points_per_segment(ind) = std::max((ssize_t)std::floor(individual_lengths(ind) / nominal_delta), (ssize_t)1);
            distance_consumed_per_segment(ind) = nominal_delta * num_points_per_segment(ind);
        }

        const ssize_t nominal_points_used = num_points_per_segment.sum();
        if (num_points < nominal_points_used + 1)
        {
            throw_arc_exception(std::runtime_error, "[ResamplePathPartialNumPoints] Unhandled edge case; more points needed than are available. Likely need to implement this using repeated splitting of the largest segment");
        }

        const ssize_t extra_points_needed = num_points - nominal_points_used - 1;
        Eigen::VectorXd dist_remaining = individual_lengths - distance_consumed_per_segment;
        for (ssize_t point = 0; point < extra_points_needed; ++point)
        {
            int ind;
            dist_remaining.maxCoeff(&ind);
            num_points_per_segment(ind)++;
            dist_remaining(ind) = 0.0;
        }

        // Build the actual path
        std::vector<Configuration, ConfigAlloc> upsampled_path(num_points);
        ssize_t next_upsampled_ind = 0;
        for (ssize_t segment_ind = 0; segment_ind < num_segments; ++segment_ind)
        {
            const double one_div_points_in_segment = 1.0 / (double)num_points_per_segment[segment_ind];
            for (ssize_t interpolation_ind = 0; interpolation_ind < num_points_per_segment[segment_ind]; ++interpolation_ind)
            {
                const double ratio = (double)interpolation_ind * one_div_points_in_segment;
                upsampled_path[next_upsampled_ind] = state_interpolation_fn(path[start_ind + segment_ind], path[start_ind + segment_ind + 1], ratio);
                next_upsampled_ind++;
            }
        }
        assert(next_upsampled_ind == num_points - 1);

        // Add the last point to terminate the path
        upsampled_path[num_points - 1] = path[end_ind - 1];

        return upsampled_path;
    }

    template<typename Configuration, typename ConfigAlloc = std::allocator<Configuration>, class DistanceFn, class InterpolationFn>
    inline std::vector<Configuration, ConfigAlloc> UpsamplePath(
            const std::vector<Configuration, ConfigAlloc>& path,
            const ssize_t num_points,
            const DistanceFn& state_distance_fn,
            const InterpolationFn& state_interpolation_fn)
    {
        return UpsamplePathPartial(path, 0, path.size(), num_points, state_distance_fn, state_interpolation_fn);
    }
}

#endif // SHORTCUT_SMOOTHING_HPP
