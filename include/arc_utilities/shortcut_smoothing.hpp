#include <functional>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>

#ifndef SHORTCUT_SMOOTHING_HPP
#define SHORTCUT_SMOOTHING_HPP

namespace shortcut_smoothing
{    
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
        // If the input data makes no sense, return an empty path
        if (end_ind >= start_ind || end_ind > path.size())
        {
            throw std::invalid_argument("Need start_ind < end_ind and end_ind <= path.size()");
        }

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
            // If there is more than one segment, interpolate between current_state and next_state (including the next_state)
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
}

#endif // SHORTCUT_SMOOTHING_HPP
