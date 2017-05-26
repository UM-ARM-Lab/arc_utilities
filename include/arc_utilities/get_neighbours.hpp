#ifndef GET_NEIGHBOURS_HPP
#define GET_NEIGHBOURS_HPP

#include <algorithm>
#include <functional>
#include <vector>

namespace arc_utilities
{
    namespace GetNeighbours
    {
        template<typename ConfigType, typename StepType, typename Allocator = std::allocator<ConfigType>>
        inline std::vector<ConfigType, Allocator> TwoDimensional8Connected(
                const ConfigType& config,
                const StepType& min_x,
                const StepType& max_x,
                const StepType& min_y,
                const StepType& max_y,
                const StepType& step_size,
                const std::function<ConfigType(const ConfigType&)>& round_to_grid_fn,
                const std::function<bool(const ConfigType&)>& validity_check_fn)
        {
            std::vector<ConfigType, Allocator> neighbours;
            neighbours.reserve(8);

            const StepType x_min = std::max(min_x, config[0] - step_size);
            const StepType x_max = std::min(max_x, config[0] + step_size);

            const StepType y_min = std::max(min_y, config[1] - step_size);
            const StepType y_max = std::min(max_y, config[1] + step_size);

            for (StepType x = x_min; x <= x_max; x += step_size)
            {
                for (StepType y = y_min; y <= y_max; y += step_size)
                {
                    if (!(x == config[0] && y == config[1]) && validity_check_fn(ConfigType(x, y)) == true)
                    {
                        neighbours.push_back(round_to_grid_fn(ConfigType(x, y)));
                    }
                }
            }

            return neighbours;
        }

        template<typename ConfigType, typename StepType, typename Allocator = std::allocator<ConfigType>>
        inline std::vector<ConfigType, Allocator> ThreeDimensional8Connected(
                const ConfigType& config,
                const StepType& min_x,
                const StepType& max_x,
                const StepType& min_y,
                const StepType& max_y,
                const StepType& min_z,
                const StepType& max_z,
                const StepType& step_size,
                const std::function<ConfigType(const ConfigType&)>& round_to_grid_fn,
                const std::function<bool(const ConfigType&)>& validity_check_fn)
        {
            std::vector<ConfigType, Allocator> neighbours;
            neighbours.reserve(26);

            const StepType x_min = std::max(min_x, config[0] - step_size);
            const StepType x_max = std::min(max_x, config[0] + step_size);

            const StepType y_min = std::max(min_y, config[1] - step_size);
            const StepType y_max = std::min(max_y, config[1] + step_size);

            const StepType z_min = std::max(min_z, config[2] - step_size);
            const StepType z_max = std::min(max_z, config[2] + step_size);

            for (StepType x = x_min; x <= x_max; x += step_size)
            {
                for (StepType y = y_min; y <= y_max; y += step_size)
                {
                    for (StepType z = z_min; z <= z_max; z += step_size)
                    {
                        if (!(x == config[0] && y == config[1] && z == config[2]) && validity_check_fn(ConfigType(x, y, z)) == true)
                        {
                            neighbours.push_back(round_to_grid_fn(ConfigType(x, y, z)));
                        }
                    }
                }
            }

            neighbours.shrink_to_fit();
            return neighbours;
        }
    }
}

#endif // GET_NEIGHBOURS_HPP
