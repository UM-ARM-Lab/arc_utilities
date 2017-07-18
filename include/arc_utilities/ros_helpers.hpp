#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "arc_utilities/maybe.hpp"

#ifndef ROS_HELPERS_HPP
#define ROS_HELPERS_HPP

namespace ROSHelpers
{
    inline void Spin(const double loop_period)
    {
        while (ros::ok())
        {
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(loop_period));
        }
    }

    template <typename T>
    inline T GetParam(ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Setting    " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParam(ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Setting    " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        return param_val;
    }


    template <typename T>
    inline T GetParamNoWarn(ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Setting    " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_INFO_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParamNoWarn(ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Setting    " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_INFO_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(40) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline Maybe::Maybe<T> GetParamRequired(ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Setting    " << std::left << std::setw(40) << param_name << " to " << param_val);
            return Maybe::Maybe<T>(param_val);
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<T>();
        }
    }
}

#endif // ROS_HELPERS_HPP
