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


void Profiler::printSummary(std::string name)
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

    std::cout << "total time : " << sum << "\n";
    std::cout << "called " << n << " times\n";
    std::cout << "min time   : "   << *std::min_element(data.begin(), data.end()) << "\n";
    std::cout << "max time   : "   << *std::min_element(data.begin(), data.end()) << "\n";
    std::cout << "average    : "   << sum/(double)n << "\n";
        
    std::cout << "\n";
}
