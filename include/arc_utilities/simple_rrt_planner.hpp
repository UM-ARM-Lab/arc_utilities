#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>

#ifndef SIMPLE_RRT_PLANNER_HPP
#define SIMPLE_RRT_PLANNER_HPP

namespace simple_rrt_planner
{
    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleRRTPlannerState
    {
    protected:

        bool initialized_;
        int64_t parent_index_;
        std::vector<int64_t> child_indices_;
        T value_;

    public:

        SimpleRRTPlannerState() : initialized_(false), parent_index_(-1)
        {
            child_indices_.clear();
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index, const std::vector<int64_t>& child_indices)
        {
            parent_index_ = parent_index;
            child_indices_ = child_indices;
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value, const int64_t parent_index)
        {
            parent_index_ = parent_index;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerState(const T& value)
        {
            parent_index_ = -1;
            child_indices_.clear();
            value_ = value;
            initialized_ = true;
        }

        const T& GetValueImmutable() const
        {
            return value_;
        }

        T& GetValueMutable()
        {
            return value_;
        }

        int64_t GetParentIndex() const
        {
            return parent_index_;
        }

        void SetParentIndex(const int64_t parent_index)
        {
            parent_index_ = parent_index;
        }

        const std::vector<int64_t>& GetChildIndices()
        {
            return child_indices_;
        }

        void ClearChildIndicies()
        {
            child_indices_.clear();
        }

        void AddChildIndex(const int64_t child_index)
        {
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] == child_index)
                {
                    return;
                }
            }
            child_indices_.push_back(child_index);
        }

        void RemoveChildIndex(const int64_t child_index)
        {
            std::vector<int64_t> new_child_indices;
            for (size_t idx = 0; idx < child_indices_.size(); idx++)
            {
                if (child_indices_[idx] != child_index)
                {
                    new_child_indices.push_back(child_indices_[idx]);
                }
            }
            child_indices_ = new_child_indices;
        }
    };

    template<typename T, typename Allocator=std::allocator<T>>
    class SimpleHybridRRTPlanner
    {
    public:

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * goal - target configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of the goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * goal_bias - in (0, 1), selects the probability that the new sampled state is the goal state
         * time_limit - limit, in seconds, for the runtime of the planner
         * rng - a Mersene-twister PRNG for internal sampling
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<T>, std::map<std::string, double>> Plan(const T& start,
                                                                      const T& goal,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&,const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& state_sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      const double goal_bias,
                                                                      const std::chrono::duration<double>& time_limit,
                                                                      std::mt19937_64& rng) const
        {
            std::uniform_real_distribution<double> goal_bias_distribution(0.0, 1.0);
            std::function<T(void)> sampling_function = [&](void) { return ((goal_bias_distribution(rng) > goal_bias) ? state_sampling_fn() : goal); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_function, forward_propagation_fn, time_limit);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * goal_sampling_fn - returns a goal state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * goal_bias - in (0, 1), selects the probability that the new sampled state is a goal state
         * time_limit - limit, in seconds, for the runtime of the planner
         * rng - a Mersene-twister PRNG for internal sampling
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<T>, std::map<std::string, double>> Plan(const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& state_sampling_fn,
                                                                      std::function<T(void)>& goal_sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      const double goal_bias,
                                                                      const std::chrono::duration<double>& time_limit,
                                                                      std::mt19937_64& rng) const
        {
            std::uniform_real_distribution<double> goal_bias_distribution(0.0, 1.0);
            std::function<T(void)> sampling_function = [&](void) { return ((goal_bias_distribution(rng) > goal_bias) ? state_sampling_fn() : goal_sampling_fn()); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_function, forward_propagation_fn, time_limit);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * time_limit - limit, in seconds, for the runtime of the planner
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<T>, std::map<std::string, double>> Plan(const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      const std::chrono::duration<double>& time_limit) const
        {
            // Keep track of time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time;
            // Keep track of states
            std::vector<SimpleRRTPlannerState<T, Allocator>> nodes;
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            nodes.push_back(start_state);
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Storage for the final planned path
            std::vector<T> planned_path;
            // Update the start time & current time
            start_time = std::chrono::high_resolution_clock::now();
            cur_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (time_limit > (cur_time - start_time))
            {
                // Sample a random goal
                T random_target = sampling_fn();
                // Get the nearest neighbor
                int64_t nearest_neighbor_index = nearest_neighbor_fn(nodes, random_target);
                const T& nearest_neighbor = nodes.at(nearest_neighbor_index).GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<T> propagated = forward_propagation_fn(nearest_neighbor, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    int64_t node_parent_index = nearest_neighbor_index;
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        const T& current_propagated = propagated[idx];
                        // Check if we've reached the goal
                        if (goal_reached_fn(current_propagated))
                        {
                            planned_path.push_back(current_propagated);
                            int64_t parent_index = node_parent_index;
                            while (parent_index >= 0)
                            {
                                const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes.at(parent_index);
                                const T& parent = parent_state.GetValueImmutable();
                                planned_path.push_back(parent);
                                parent_index = parent_state.GetParentIndex();
                            }
                            std::reverse(planned_path.begin(), planned_path.end());
                            // Update the statistics
                            cur_time = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> planning_time(cur_time - start_time);
                            statistics["planning_time"] = planning_time.count();
                            statistics["total_states"] = nodes.size();
                            statistics["solution_path_length"] = planned_path.size();
                            // Put together the results
                            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, statistics);
                        }
                        // If not, add it to the tree
                        else
                        {
                            SimpleRRTPlannerState<T, Allocator> new_state(current_propagated, node_parent_index);
                            nodes.push_back(new_state);
                            node_parent_index = (int64_t)nodes.size() - 1;
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
                // Update the end time
                cur_time = std::chrono::high_resolution_clock::now();
            }
            std::cerr << "Planning time exceeded" << std::endl;
            // Put together the results
            cur_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time(cur_time - start_time);
            statistics["planning_time"] = planning_time.count();
            statistics["total_states"] = nodes.size();
            statistics["solution_path_length"] = 0.0;
            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, statistics);
        }
    };

}

#endif // SIMPLE_RRT_PLANNER
