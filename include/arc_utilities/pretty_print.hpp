#include <iostream>
#include <array>
#include <vector>
#include <deque>
#include <forward_list>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <Eigen/Geometry>

#ifndef PRETTY_PRINT_HPP
#define PRETTY_PRINT_HPP

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

// Handy functions for printing vectors and pairs
namespace PrettyPrint
{

    // Base template function for printing types
    template <typename T>
    inline std::string PrettyPrint(const T& toprint, const bool add_delimiters=false)
    {
        UNUSED(add_delimiters);
        std::ostringstream strm;
        strm << toprint;
        return strm.str();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Specializations for specific types - if you want a specialization for a new type, add it here /////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    template<>
    inline std::string PrettyPrint(const bool& bool_to_print, const bool add_delimiters)
    {
        UNUSED(add_delimiters);
        if (bool_to_print)
        {
            return "true";
        }
        else
        {
            return "false";
        }
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Vector3d& vector_to_print, const bool add_delimiters)
    {
        UNUSED(add_delimiters);
        return "Vector3d: <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">";
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Quaterniond& quaternion_to_print, const bool add_delimiters)
    {
        UNUSED(add_delimiters);
        return "Quaterniond <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
    }

    template<>
    inline std::string PrettyPrint(const Eigen::Affine3d& transform_to_print, const bool add_delimiters)
    {
        UNUSED(add_delimiters);
        Eigen::Vector3d vector_to_print = transform_to_print.translation();
        Eigen::Quaterniond quaternion_to_print(transform_to_print.rotation());
        return "Affine3d <x: " + std::to_string(vector_to_print.x()) + " y: " + std::to_string(vector_to_print.y()) + " z: " + std::to_string(vector_to_print.z()) + ">, <x: " + std::to_string(quaternion_to_print.x()) + " y: " + std::to_string(quaternion_to_print.y()) + " z: " + std::to_string(quaternion_to_print.z()) + " w: " + std::to_string(quaternion_to_print.w()) + ">";
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///// Template functions for printing containers - if you want to add a new container, add your function below /////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    template <typename T, size_t N>
    inline std::string PrettyPrint(const std::array<T, N>& arraytoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (arraytoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[" << PrettyPrint(arraytoprint[0], add_delimiters);
                for (size_t idx = 1; idx < arraytoprint.size(); idx++)
                {
                    strm << ", " << PrettyPrint(arraytoprint[idx], add_delimiters);
                }
                strm << "]";
            }
            else
            {
                strm << PrettyPrint(arraytoprint[0], add_delimiters);
                for (size_t idx = 1; idx < arraytoprint.size(); idx++)
                {
                    strm << ", " << PrettyPrint(arraytoprint[idx], add_delimiters);
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::vector<T>& vectoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (vectoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[" << PrettyPrint(vectoprint[0], add_delimiters);
                for (size_t idx = 1; idx < vectoprint.size(); idx++)
                {
                    strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
                }
                strm << "]";
            }
            else
            {
                strm << PrettyPrint(vectoprint[0], add_delimiters);
                for (size_t idx = 1; idx < vectoprint.size(); idx++)
                {
                    strm << ", " << PrettyPrint(vectoprint[idx], add_delimiters);
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::list<T>& listtoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (listtoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::list<T>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::list<T>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::forward_list<T>& listtoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (listtoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::forward_list<T>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::forward_list<T>::const_iterator itr;
                for (itr = listtoprint.begin(); itr != listtoprint.end(); ++itr)
                {
                    if (itr != listtoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::deque<T>& dequetoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (dequetoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "[";
                typename std::deque<T>::const_iterator itr;
                for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
                {
                    if (itr != dequetoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << "]";
            }
            else
            {
                typename std::deque<T>::const_iterator itr;
                for (itr = dequetoprint.begin(); itr != dequetoprint.end(); ++itr)
                {
                    if (itr != dequetoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::pair<A, B>& pairtoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (add_delimiters)
        {
            strm << "<" << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters) << ">";
        }
        else
        {
            strm << PrettyPrint(pairtoprint.first, add_delimiters) << ": " << PrettyPrint(pairtoprint.second, add_delimiters);
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::map<A, B>& maptoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::map<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::map<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::multimap<A, B>& maptoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::multimap<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::multimap<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::set<T>& settoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::set<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::set<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::multiset<T>& settoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::multiset<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::multiset<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::unordered_map<A, B>& maptoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::unordered_map<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::unordered_map<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename A, typename B>
    inline std::string PrettyPrint(const std::unordered_multimap<A, B>& maptoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (maptoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "{";
                typename std::unordered_multimap<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
                strm << "}";
            }
            else
            {
                typename std::unordered_multimap<A, B>::const_iterator itr;
                for (itr = maptoprint.begin(); itr != maptoprint.end(); ++itr)
                {
                    std::pair<A, B> cur_pair(itr->first, itr->second);
                    if (itr != maptoprint.begin())
                    {
                        strm << ", " << PrettyPrint(cur_pair, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(cur_pair, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::unordered_set<T>& settoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::unordered_set<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::unordered_set<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }

    template <typename T>
    inline std::string PrettyPrint(const std::unordered_multiset<T>& settoprint, const bool add_delimiters=false)
    {
        std::ostringstream strm;
        if (settoprint.size() > 0)
        {
            if (add_delimiters)
            {
                strm << "(";
                typename std::unordered_multiset<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
                strm << ")";
            }
            else
            {
                typename std::unordered_multiset<T>::const_iterator itr;
                for (itr = settoprint.begin(); itr != settoprint.end(); ++itr)
                {
                    if (itr != settoprint.begin())
                    {
                        strm << ", " << PrettyPrint(*itr, add_delimiters);
                    }
                    else
                    {
                        strm << PrettyPrint(*itr, add_delimiters);
                    }
                }
            }
        }
        return strm.str();
    }
}

#endif // PRETTY_PRINT_HPP
