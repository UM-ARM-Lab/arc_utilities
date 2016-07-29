#include <ros/ros.h>
#include <ros/callback_queue.h>

#ifndef ROS_HELPERS_HPP
#define ROS_HELPERS_HPP

namespace ROSHelpers
{
    inline Spin(const double loop_period)
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
            ROS_INFO_STREAM("Setting " << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM(param_name << " not set! Using default of " << param_val);
        }
        return param_val;
    }

    template <typename T>
    inline T GetParam(ros::NodeHandle& nh, const std::string& param_name, T&& default_val)
    {
        T param_val;
        if (nh.getParam(param_name, param_val))
        {
            ROS_INFO_STREAM("Setting " << param_name << " to " << param_val);
        }
        else
        {
            param_val = default_val;
            ROS_WARN_STREAM(param_name << " not set! Using default of " << param_val);
        }
        return param_val;
    }
}

#endif // ROS_HELPERS_HPP
