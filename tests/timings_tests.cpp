#include "arc_utilities/timing.hpp"
#include <gtest/gtest.h>


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
    Profiler::initialize(10,10);
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

TEST(TimerTest, MultiTimer)
{
    Profiler::initialize(10,10);
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
    Profiler::initialize(10,10);
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



GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

