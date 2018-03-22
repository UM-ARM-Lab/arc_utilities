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
namespace{

TEST(TimerTest, Functioning)
{
    Profiler::initialize(10,10);
    Profiler::startTimer("timer1");
    double t_elapsed = Profiler::record("timer1");
    EXPECT_TRUE(t_elapsed > 0);

    for(size_t i=2; i<100; i++)
    {
        Profiler::record("timer1");
        
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



GTEST_API_ int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

