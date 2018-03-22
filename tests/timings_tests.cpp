
#define ENABLE_PROFILING
#include "arc_utilities/timing.hpp"
#include <gtest/gtest.h>
#include <thread>


// using ::testing::EmptyTestEventListener;
// using ::testing::InitGoogleTest;
// using ::testing::Test;
// using ::testing::TestCase;
// using ::testing::TestEventListeners;
// using ::testing::TestInfo;
// using ::testing::TestPartResult;
// using ::testing::UnitTest;


using namespace arc_utilities;



TEST(TimerTest, Functioning)
{
    Profiler::reset_and_preallocate(10,10);
    Profiler::startTimer("timer1");
    double t_elapsed = Profiler::record("timer1");
    EXPECT_TRUE(t_elapsed > 0);

    for(size_t i=2; i<100; i++)
    {
        Profiler::record("timer1");
        std::vector<double> times = Profiler::getData("timer1");
        ASSERT_EQ(i, times.size());
        EXPECT_TRUE(times[i-1] > times[i-2]);
    }

}

TEST(TimerTest, TimerAccuracy)
{
    Profiler::startTimer("sleepy");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    double t_elapsed = Profiler::record("sleepy");
    EXPECT_TRUE(t_elapsed < 0.051);
    EXPECT_TRUE(t_elapsed > 0.0500);
}

TEST(TimerTest, MultiTimer)
{
    Profiler::reset_and_preallocate(10,10);
    Profiler::startTimer("timer1");
    Profiler::startTimer("timer2");
    double t2_elapsed = Profiler::record("timer2");
    double t1_elapsed = Profiler::record("timer1");
    EXPECT_TRUE(t1_elapsed > t2_elapsed);
    EXPECT_TRUE(t1_elapsed > 0);
    EXPECT_TRUE(t2_elapsed > 0);
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
    EXPECT_TRUE(t1_elapsed_restarted < t2_elapsed);

    Profiler::startTimer("timer2");
    double t2_elapsed_restarted = Profiler::record("timer2");
    EXPECT_TRUE(t1_elapsed > t2_elapsed_restarted);

}

TEST(TimerTest, Macros)
{
    PROFILE_RESET_ALL(100,100);
    PROFILE_START("testmacro1");
    PROFILE_START("testmacro2");
    PROFILE_RECORD("testmacro2");
    PROFILE_RECORD("testmacro1");

    // PROFILE_PRINT_SUMMARY_FOR_GROUP("testmacro1");
    std::vector<std::string> names = {"testmacro1", "testmacro2"};
    PROFILE_PRINT_SUMMARY_FOR_GROUP(names);

    double t1_elapsed = Profiler::getData("testmacro1")[0];
    double t2_elapsed = Profiler::getData("testmacro2")[0];
    EXPECT_TRUE(t1_elapsed > t2_elapsed);

}


GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

