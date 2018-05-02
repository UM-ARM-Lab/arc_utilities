
#define ENABLE_PROFILING
#include "arc_utilities/timing.hpp"
#include <gtest/gtest.h>
#include <thread>


using namespace arc_utilities;



TEST(TimerTest, Functioning)
{
    Profiler::reset_and_preallocate(10,10);
    Profiler::startTimer("timer1");
    double t_elapsed = Profiler::record("timer1");
    EXPECT_TRUE(t_elapsed > 0) << "Timer recorded negative elapsed time";

    for(size_t i=2; i<100; i++)
    {
        Profiler::record("timer1");
        std::vector<double> times = Profiler::getData("timer1");
        ASSERT_EQ(i, times.size()) << "Num events and number of times getData called do not match";
        EXPECT_TRUE(times[i-1] > times[i-2]) << "Time not monotonitcally increasing";
    }

}

TEST(TimerTest, TimerAccuracy)
{
    Profiler::startTimer("sleepy");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    double t_elapsed = Profiler::record("sleepy");
    EXPECT_TRUE(t_elapsed < 0.052) << "Sleep for 50ms took too long";
    EXPECT_TRUE(t_elapsed > 0.0500) << "Recorded time less than sleep time";
}

TEST(TimerTest, MultiTimer)
{
    Profiler::reset_and_preallocate(10,10);
    Profiler::startTimer("timer1");
    Profiler::startTimer("timer2");
    double t2_elapsed = Profiler::record("timer2");
    double t1_elapsed = Profiler::record("timer1");
    EXPECT_TRUE(t1_elapsed > t2_elapsed) << "Timer1 started first but not registering longer time";
    EXPECT_TRUE(t1_elapsed > 0) << "Timer 1 recorded non-positive elapsed time";
    EXPECT_TRUE(t2_elapsed > 0) << "Timer 2 recorded non-positive elapsed time";
}

TEST(TimerTest, RestartingTimer)
{
    Profiler::reset_and_preallocate(10,10);
    Profiler::startTimer("timer1");
    Profiler::startTimer("timer2");
    double t2_elapsed = Profiler::record("timer2");
    double t1_elapsed = Profiler::record("timer1");


    Profiler::startTimer("timer1");
    double t1_elapsed_restarted = Profiler::record("timer1");
    EXPECT_TRUE(t1_elapsed_restarted < t2_elapsed) << "Restarted timer 1, but long time measured";

    Profiler::startTimer("timer2");
    double t2_elapsed_restarted = Profiler::record("timer2");
    EXPECT_TRUE(t1_elapsed > t2_elapsed_restarted) << "Restarted timer2 but long time measured";

}

TEST(TimerTest, Macros)
{
    PROFILE_REINITIALIZE(100,100);
    PROFILE_START("testmacro1");
    PROFILE_START("testmacro2");
    PROFILE_RECORD("testmacro2");
    PROFILE_RECORD("testmacro1");

    // PROFILE_PRINT_SUMMARY_FOR_GROUP("testmacro1");

    double t1_elapsed = Profiler::getData("testmacro1")[0];
    double t2_elapsed = Profiler::getData("testmacro2")[0];
    EXPECT_TRUE(t1_elapsed > t2_elapsed) << "Macro timer1 started first but recorded less time";
}

TEST(TimerTest, Printing)
{
    PROFILE_REINITIALIZE(100,1000);
    PROFILE_START("name_01");
    PROFILE_START("name_02");
    PROFILE_START("really really really long name");
    PROFILE_RECORD("testmacro2");
    PROFILE_RECORD("testmacro1");
    for(int i=0; i<100; i++)
    {
        PROFILE_RECORD("really really really long name");
    }
    std::vector<std::string> names = {"name_01", "unused name",
                                      "~~~~~~~~~~~~~~",
                                      "really really really long name",
                                      "name_02"};

    PROFILE_PRINT_SUMMARY_FOR_GROUP(names);
}


TEST(TimerTest, WritingSummary)
{
    PROFILE_REINITIALIZE(100, 100);
    PROFILE_START("one");
    PROFILE_START("two");
    PROFILE_START("three");
    PROFILE_START("double values");
    PROFILE_RECORD("one");
    PROFILE_RECORD("two");
    PROFILE_RECORD("three");
    PROFILE_RECORD_DOUBLE("double values", 3.2);
    PROFILE_RECORD_DOUBLE("double values", -1.966);

    PROFILE_WRITE_SUMMARY_FOR_ALL("testing_output.txt");
}


TEST(TimerTest, WritingAll)
{
    PROFILE_REINITIALIZE(100, 100);
    PROFILE_START("one");
    PROFILE_START("two");
    PROFILE_START("three");
    PROFILE_START("double values");
    PROFILE_RECORD("one");
    PROFILE_RECORD("two");
    PROFILE_RECORD("three");
    PROFILE_RECORD_DOUBLE("double values", 1.1);
    PROFILE_RECORD_DOUBLE("double values", 2.2);
    PROFILE_RECORD_DOUBLE("double values", 3.3);
    PROFILE_WRITE_ALL("testing_output_full.txt");
}


GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

