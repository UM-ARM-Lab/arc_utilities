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

        const std::vector<int64_t>& GetChildIndices() const
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
    class SimpleRRTPlannerPointerState
    {
    protected:

        bool initialized_;
        std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>> parent_;
        T value_;

    public:

        SimpleRRTPlannerPointerState() : initialized_(false)
        {
            parent_ = std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>();
        }

        SimpleRRTPlannerPointerState(const T& value, const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& parent)
        {
            parent_(parent);
            value_ = value;
            initialized_ = true;
        }

        SimpleRRTPlannerPointerState(const T& value)
        {
            parent_ = std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>();
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

        const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& GetParent() const
        {
            return parent_;
        }

        void SetParent(const std::shared_ptr<const SimpleRRTPlannerPointerState<T, Allocator>>& parent)
        {
            parent_(parent);
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
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            std::function<bool(void)> termination_check_fn = [&](void) { return (((std::chrono::time_point<std::chrono::high_resolution_clock>)std::chrono::high_resolution_clock::now() - start_time) > time_limit); };
            return Plan(start, nearest_neighbor_fn, goal_reached_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
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
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
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
                                                                      std::function<bool(void)>& termination_check_fn) const
        {
            // Define a couple lambdas to let us use the generic multi-path planner as if it were a single-path planner
            bool solution_found = false;
            std::function<bool(const T&)> real_goal_found_fn = [&](const T& state) { if (goal_reached_fn(state)) { solution_found = true; return true; } else {return false;} };
            std::function<bool(void)> real_termination_check_fn = [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            std::function<void(const SimpleRRTPlannerState<T, Allocator>&)> dummy_goal_callback_fn = [](const SimpleRRTPlannerState<T, Allocator>& state) {;};
            // Call the planner
            std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> planning_result = PlanMultiPath(start, nearest_neighbor_fn, real_goal_found_fn, dummy_goal_callback_fn, sampling_fn, forward_propagation_fn, real_termination_check_fn);
            // Put together the return
            std::vector<T> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, planning_result.second);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<path, statistics>
         * path - vector of states corresponding to the planned path
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<T>, std::map<std::string, double>> Plan(std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                                                                      const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn) const
        {
            // Define a couple lambdas to let us use the generic multi-path planner as if it were a single-path planner
            bool solution_found = false;
            std::function<bool(const T&)> real_goal_found_fn = [&](const T& state) { if (goal_reached_fn(state)) { solution_found = true; return true; } else {return false;} };
            std::function<bool(void)> real_termination_check_fn = [&](void) { if (!solution_found) { return termination_check_fn(); } else {return true;} };
            std::function<void(const SimpleRRTPlannerState<T, Allocator>&)> dummy_goal_callback_fn = [](const SimpleRRTPlannerState<T, Allocator>& state) {;};
            // Call the planner
            std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> planning_result = PlanMultiPath(nodes, start, nearest_neighbor_fn, real_goal_found_fn, dummy_goal_callback_fn, sampling_fn, forward_propagation_fn, real_termination_check_fn);
            // Put together the return
            std::vector<T> planned_path;
            if (planning_result.first.size() > 0)
            {
                planned_path = planning_result.first[0];
            }
            return std::pair<std::vector<T>, std::map<std::string, double>>(planned_path, planning_result.second);
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
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<void(SimpleRRTPlannerState<T, Allocator>&)>& goal_reached_callback_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn) const
        {
            // Keep track of states
            std::vector<SimpleRRTPlannerState<T, Allocator>> nodes;
            return PlanMultiPath(nodes, start, nearest_neighbor_fn, goal_reached_fn, goal_reached_callback_fn, sampling_fn, forward_propagation_fn, termination_check_fn);
        }

        /* Template-based single-tree RRT planner
         *
         * Template type T is your state type (i.e. a configuration)
         *
         * Arguments:
         * nodes - a mutable vector of planner states, used internally to store the planner tree.
         *          This is provided to allow external use of the tree during and after planning.
         * start - starting configuration
         * nearest_neighbor_fn - given all nodes explored so far, and a new state, return the index of the "closest" node
         * goal_reached_fn - return if a given state meets the goal conditions (for example, within a radius of a goal state)
         * state_sampling_fn - returns a new state (randomly- or deterministically-sampled)
         * forward_propagation_fn - given the nearest neighbor and a new target state, returns the states that would grow the tree towards the target
         * termination_check_fn - returns if the planner should terminate (for example, if it has exceeded time/space limits)
         *
         * Returns:
         * std::pair<paths, statistics>
         * paths - vector of vector of states corresponding to the planned path(s)
         * statistics - map of string keys/double values of planner statistics (i.e. run time, #states explored, #states in solution
         */
        std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes,
                                                                      const T& start,
                                                                      std::function<int64_t(const std::vector<SimpleRRTPlannerState<T, Allocator>>&, const T&)>& nearest_neighbor_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<void(SimpleRRTPlannerState<T, Allocator>&)>& goal_reached_callback_fn,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn) const
        {
            // Clear the tree we've been given
            nodes.clear();
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            nodes.push_back(start_state);
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Storage for the goal states we reach
            std::vector<int64_t> goal_state_indices;
            // Safety check before doing real work
            if (goal_reached_fn(start))
            {
                goal_state_indices.push_back(0);
                std::cerr << "Start state meets goal conditions, returning default path [start]" << std::endl;
                // Put together the results
                std::vector<std::vector<T>> planned_paths = ExtractSolutionPaths(nodes, goal_state_indices);
                statistics["planning_time"] = 0.0;
                statistics["total_states"] = nodes.size();
                statistics["solutions"] = (double)planned_paths.size();
                return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
            }
            // Update the start time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Sample a random goal
                T random_target = sampling_fn();
                // Get the nearest neighbor
                int64_t nearest_neighbor_index = nearest_neighbor_fn(nodes, random_target);
                assert(nearest_neighbor_index >= 0);
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
                        SimpleRRTPlannerState<T, Allocator> new_state(current_propagated, node_parent_index);
                        nodes.push_back(new_state);
                        int64_t new_node_index = (int64_t)nodes.size() - 1;
                        nodes[node_parent_index].AddChildIndex(new_node_index);
                        // Check if we've reached the goal
                        if (goal_reached_fn(current_propagated))
                        {
                            goal_state_indices.push_back(new_node_index);
                            goal_reached_callback_fn(nodes[nodes.size() - 1]);
                            break;
                        }
                        // If not, add it to the tree
                        else
                        {
                            node_parent_index = new_node_index;
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
            }
            std::cout << "Planner termination condition met" << std::endl;
            // Put together the results
            std::vector<std::vector<T>> planned_paths = ExtractSolutionPaths(nodes, goal_state_indices);
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time(cur_time - start_time);
            statistics["planning_time"] = planning_time.count();
            statistics["total_states"] = nodes.size();
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
        }

        /* Extracts all the solution paths corresponding to the provided goal states
         */
        std::vector<std::vector<T>> ExtractSolutionPaths(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes, const std::vector<int64_t>& goal_state_indices) const
        {
            std::vector<std::vector<T>> solution_paths;
            for (size_t idx = 0; idx < goal_state_indices.size(); idx++)
            {
                std::vector<T> solution_path = ExtractSolutionPath(nodes, goal_state_indices[idx]);
                solution_paths.push_back(solution_path);
            }
            return solution_paths;
        }

        /* Extracts a single solution path corresponding to the provided goal state
         */
        std::vector<T> ExtractSolutionPath(const std::vector<SimpleRRTPlannerState<T, Allocator>>& nodes, const int64_t goal_state_index) const
        {
            std::vector<T> solution_path;
            const SimpleRRTPlannerState<T, Allocator>& goal_state = nodes[goal_state_index];
            solution_path.push_back(goal_state.GetValueImmutable());
            int64_t parent_index = goal_state.GetParentIndex();
            while (parent_index >= 0)
            {
                assert(parent_index < nodes.size());
                const SimpleRRTPlannerState<T, Allocator>& parent_state = nodes[parent_index];
                const T& parent = parent_state.GetValueImmutable();
                solution_path.push_back(parent);
                parent_index = parent_state.GetParentIndex();
            }
            // Put it in the right order
            std::reverse(solution_path.begin(), solution_path.end());
            return solution_path;
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
        std::pair<std::vector<std::vector<T>>, std::map<std::string, double>> PlanMultiPath(const T& start,
                                                                      std::function<void(const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&)>& register_nearest_neighbors_fn,
                                                                      std::function<const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&(const T&)>& get_nearest_neighbor_fn,
                                                                      std::function<std::vector<std::vector<T>>(void)>& extract_solution_paths,
                                                                      std::function<T(void)>& sampling_fn,
                                                                      std::function<bool(const T&)>& goal_reached_fn,
                                                                      std::function<void(const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>&)>& register_goal_state_fn,
                                                                      std::function<std::vector<T>(const T&, const T&)>& forward_propagation_fn,
                                                                      std::function<bool(void)>& termination_check_fn) const
        {
            // Keep track of statistics
            std::map<std::string, double> statistics;
            statistics["total_states"] = 0.0;
            statistics["total_samples"] = 0.0;
            statistics["successful_samples"] = 0.0;
            statistics["failed_samples"] = 0.0;
            // Add the start state
            SimpleRRTPlannerState<T, Allocator> start_state(start);
            register_nearest_neighbors_fn(start_state);
            // Update the start time
            std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
            // Plan
            while (!termination_check_fn())
            {
                // Sample a random goal
                T random_target = sampling_fn();
                // Get the nearest neighbor
                const std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>>& nearest_neighbor_ptr = get_nearest_neighbor_fn(random_target);
                assert(nearest_neighbor_ptr);
                const T& nearest_neighbor_value = nearest_neighbor_ptr->GetValueImmutable();
                // Forward propagate towards the goal
                std::vector<T> propagated = forward_propagation_fn(nearest_neighbor_value, random_target);
                if (!propagated.empty())
                {
                    statistics["total_samples"] += 1.0;
                    statistics["successful_samples"] += 1.0;
                    std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>> parent_ptr(nearest_neighbor_ptr);
                    for (size_t idx = 0; idx < propagated.size(); idx++)
                    {
                        statistics["total_states"] += 1.0;
                        const T& current_propagated = propagated[idx];
                        std::shared_ptr<SimpleRRTPlannerPointerState<T, Allocator>> new_state_ptr(new SimpleRRTPlannerPointerState<T, Allocator>(current_propagated, parent_ptr));
                        // If we've reached a goal, register it specially
                        if (goal_reached_fn(current_propagated))
                        {
                            register_goal_state_fn(new_state_ptr);
                            break;
                        }
                        // Otherwise, simply register it as a nearest neighbor
                        else
                        {
                            register_nearest_neighbors_fn(new_state_ptr);
                            parent_ptr = new_state_ptr;
                        }
                    }
                }
                else
                {
                    statistics["total_samples"] += 1.0;
                    statistics["failed_samples"] += 1.0;
                }
            }
            // Put together the results
            std::chrono::time_point<std::chrono::high_resolution_clock> cur_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> planning_time(cur_time - start_time);
            std::vector<std::vector<T>> planned_paths = extract_solution_paths();
            statistics["planning_time"] = planning_time.count();
            statistics["solutions"] = (double)planned_paths.size();
            return std::pair<std::vector<std::vector<T>>, std::map<std::string, double>>(planned_paths, statistics);
        }
    };

}

#endif // SIMPLE_RRT_PLANNER
