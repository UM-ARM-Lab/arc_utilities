#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "arc_utilities/maybe.hpp"

#ifndef ROS_HELPERS_HPP
#define ROS_HELPERS_HPP

namespace ROSHelpers
{
    static auto constexpr PARAM_NAME_WIDTH = 50;

    inline void Spin(const double loop_period)
    {
        while (ros::ok())
        {
            ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(loop_period));
        }
    }

    template <typename T>
    inline T GetParam(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParam(const ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParamDebugLog(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_DEBUG_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_DEBUG_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParamDebugLog(const ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_DEBUG_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_DEBUG_STREAM_NAMED("params", "Defaulting " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " to " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline Maybe::Maybe<T> GetParamOptional(const ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
            return Maybe::Maybe<T>(param_val);
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<T>();
        }
    }

    template <typename T>
    inline Maybe::Maybe<T> GetParamRequired(const ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
            return Maybe::Maybe<T>(param_val);
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<T>();
        }
    }

    template <typename T>
    inline Maybe::Maybe<T> GetParamRequiredDebugLog(const ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_DEBUG_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " as " << param_val);
            return Maybe::Maybe<T>(param_val);
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<T>();
        }
    }

    template <typename T>
    inline Maybe::Maybe<std::vector<T>> GetVectorOptional(const ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        std::vector<T> vec;
        if (nh.getParam(param_name, vec))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " of length " << vec.size());
            return Maybe::Maybe<std::vector<T>>(vec);
        }
        else
        {
            ROS_INFO_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<std::vector<T>>();
        }
    }

    template <typename T>
    inline Maybe::Maybe<std::vector<T>> GetVectorRequired(const ros::NodeHandle& nh, const std::string& param_name, const std::string& calling_fn_name)
    {
        ROS_DEBUG_STREAM_NAMED("params", "No default value for " << param_name << ": Value must be on paramter sever");
        std::vector<T> vec;
        if (nh.getParam(param_name, vec))
        {
            ROS_INFO_STREAM_NAMED("params", "Retrieving " << std::left << std::setw(PARAM_NAME_WIDTH) << param_name << " of length " << vec.size());
            return Maybe::Maybe<std::vector<T>>(vec);
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << param_name << " on parameter server for " << calling_fn_name << ": Value must be on paramter sever");
            return Maybe::Maybe<std::vector<T>>();
        }
    }
}

#endif // ROS_HELPERS_HPP
