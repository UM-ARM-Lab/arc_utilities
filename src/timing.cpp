#include "arc_utilities/timing.hpp"
#include <iostream>
#include <cassert>
#include <algorithm>

using namespace arc_utilities;

double GlobalStopwatch(const StopwatchControl control)
{
    static Stopwatch global_stopwatch;
    return global_stopwatch(control);
}




Profiler* Profiler::m_instance = NULL;

Profiler* Profiler::getInstance()
{
    if (m_instance == NULL)
    {
        m_instance = new Profiler();
    }
    return m_instance;
}

void Profiler::reset_and_preallocate(size_t num_names, size_t num_events)
{
    Profiler* monitor = getInstance();
    monitor->data.clear();
    monitor->prealloc_buffer.resize(num_names);
    for (size_t i=0; i<num_names; i++)
    {
        monitor->prealloc_buffer[i].reserve(num_events);
    }
}

void Profiler::reset(std::string name)
{
    Profiler* m = getInstance();
    if(m->data.find(name) != m->data.end())
    {
        m->data[name].resize(0);
        startTimer(name);
    }
        
}

void Profiler::addData(std::string name, double datum)
{
    Profiler* m = getInstance();
    if (m->data.find(name) == m->data.end())
    {
        m->data[name] = std::vector<double>();
        if (m->prealloc_buffer.size() > 0)
        {
            m->data[name].swap(m->prealloc_buffer.back());
            m->prealloc_buffer.pop_back();
        }
    }
    m->data[name].push_back(datum);
}


void Profiler::startTimer(std::string timer_name)
{
    Profiler* m = getInstance();
    if (m->timers.find(timer_name) == m->timers.end())
    {
        m->timers[timer_name] = Stopwatch();
    }
    m->timers[timer_name](RESET);
}


double Profiler::record(std::string timer_name)
{
    Profiler* m = getInstance();
    if (m->timers.find(timer_name) == m->timers.end())
    {
        std::cout << "Attempting to record timer \""<< timer_name <<
            "\" before timer started\n";
        assert(false);
    }
    double time_elapsed = m->timers[timer_name]();
    m->addData(timer_name, time_elapsed);
    return time_elapsed;
}

std::vector<double> Profiler::getData(std::string name)
{
    Profiler* m = getInstance();
    return m->data[name];
}


void Profiler::printSingleSummary(std::string name)
{
    Profiler* m = getInstance();
    std::string box = std::string(2+name.length(), '=');
    std::cout << " ." << box << ". " << "\n";
    std::cout << "|| " << name << " || Summary :\n";
    std::cout << " '" << box << "' " << "\n";
    if (m->data.find(name) == m->data.end())
    {
        std::cout << name << " never called\n\n";
        return;
    }

    std::vector<double> data = m->getData(name);

    size_t n = data.size();
    double sum = 0;
    for(auto& num : data)
    {
        sum += num;
    }

    std::cout << "total time : " << sum << " s\n";
    std::cout << "called " << n << " times\n";
    std::cout << "min time   : "   << *std::min_element(data.begin(), data.end()) << "s\n";
    std::cout << "max time   : "   << *std::max_element(data.begin(), data.end()) << "s\n";
    std::cout << "average    : "   << sum/(double)n << "s\n";
        
    std::cout << "\n";
}

void Profiler::printGroupSummary(const std::vector<std::string> &names)
{

    Profiler* m = getInstance();
    std::cout << " .=======================. \n";
    std::cout << "||    Profile Summary    ||\n";
    std::cout << " '=======================' \n";


    std::size_t label_len = max_element(names.begin(), names.end(),
                                        [] (const std::string &a, const std::string &b)
                                        {return a.length() < b.length();}) -> length() + 2;

    label_len = std::max(label_len, (size_t)8);

    const std::string label_format = ("%-" + std::to_string(label_len) + "s");
    
    printf(label_format.c_str(), "Label");
    printf("%16s", "tot time (s)");
    printf("%16s", "num_calls");
    printf("%16s", "avg time (s)");
    printf("\n");

    std::string seperator = std::string(label_len-2, '~') + "      " + std::string(12, '.')
        + "       " + std::string(9, '~') + "    " + std::string(12, '.');

    for(const auto& name: names)
    {
        if(name.find("~~~~~")==0)
        {

            printf("%s\n", seperator.c_str());
            continue;
        }
        printf(label_format.c_str(), name.c_str());
        double tot_time = 0.0;
        double avg_time = 0.0;
        size_t num_calls = 0;
        if(m->data.find(name) != m->data.end())
        {
            std::vector<double> &data = m->data[name];
            tot_time = 0;
            for(auto& val : data)
            {
                tot_time += val;
            }
            num_calls = data.size();
            avg_time = tot_time/(double)num_calls;
        }

        printf(" %15f %15ld %15f\n", tot_time, num_calls, avg_time);
    }
}
