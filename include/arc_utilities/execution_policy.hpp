#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <arc_utilities/pretty_print.hpp>

#ifndef EXECUTION_POLICY_HPP
#define EXECUTION_POLICY_HPP

namespace execution_policy
{
    template<typename Observation, typename Action>
    class ExecutionPolicy
    {
    protected:

        bool initialized_;
        std::vector<std::pair<std::pair<Observation, Action>, double>> policy_;
        std::function<double(const Observation&, const Observation&)> distance_fn_;

    public:

        inline ExecutionPolicy(const std::function<double(const Observation&, const Observation&)>& distance_fn) : initialized_(true), distance_fn_(distance_fn) {}

        inline ExecutionPolicy() : initialized_(false), distance_fn_([] (const Observation&, const Observation&) { return INFINITY; }) {}

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        inline void ExtendPolicy(const Observation& observation, const Action& action, const double confidence)
        {
            assert(initialized_);
            assert(confidence > 0.0);
            assert(confidence <= 1.0);
            policy_.push_back(std::pair<std::pair<Observation, Action>, double>(std::pair<Observation, Action>(observation, action), confidence));
        }

        inline std::pair<Action, std::pair<double, double>> GetAction(const Observation& observation, const double max_distance) const
        {
            assert(initialized_);
            double min_dist = INFINITY;
            double best_confidence = 0.0;
            Action best_action;
            for (size_t idx = 0; idx < policy_.size(); idx++)
            {
                const std::pair<Observation, Action>& candidate = policy_[idx].first;
                const double& confidence = policy_[idx].second;
                const double raw_distance = distance_fn_(observation, candidate.first);
                if (raw_distance <= fabs(max_distance))
                {
                    const double confidence_weight = 1.0 / confidence;
                    const double weighted_distance = raw_distance * confidence_weight;
                    if (weighted_distance < min_dist)
                    {
                        min_dist = raw_distance;
                        best_confidence = confidence;
                        best_action = candidate.second;
                    }
                }
            }
            return std::pair<Action, std::pair<double, double>>(best_action, std::pair<double, double>(min_dist, best_confidence));
        }

        inline const std::vector<std::pair<std::pair<Observation, Action>, double>>& GetRawPolicy() const
        {
            return policy_;
        }
    };
}

template<typename Observation, typename Action>
std::ostream& operator<<(std::ostream& strm, const execution_policy::ExecutionPolicy<Observation, Action>& policy)
{
    const std::vector<std::pair<std::pair<Observation, Action>, double>>& raw_policy = policy.GetRawPolicy();
    strm << "Execution Policy - Policy: ";
    for (size_t idx = 0; idx < raw_policy.size(); idx++)
    {
        const std::pair<Observation, Action>& observation_action_pair = raw_policy[idx].first;
        const double& pair_confidence = raw_policy[idx].second;
        strm << "\nObservation: " << PrettyPrint::PrettyPrint(observation_action_pair.first) << " | Action: " << PrettyPrint::PrettyPrint(observation_action_pair.second) << " | Confidence: " << pair_confidence;
    }
    return strm;
}

#endif // EXECUTION_POLICY_HPP
