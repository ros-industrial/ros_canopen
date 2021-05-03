// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/dummy.h>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(DummyInterfaceTest, testCase1)
{
    can::DummyBus bus("testCase1");
    can::ThreadedDummyInterface dummy;
    dummy.init(bus.name, true, can::NoSettings::create());

    can::DummyReplay replay;
    replay.add("0#8200", {"701#00", "701#04"});
    replay.init(bus);

    std::list<std::string> expected{"0#8200", "701#00", "701#04"};
    std::list<std::string> responses;

    auto listener = dummy.createMsgListener([&responses](auto& f) {
        responses.push_back(can::tostring(f, true));
    });

    EXPECT_FALSE(replay.done());

    dummy.send(can::toframe("0#8200"));

    replay.flush();
    dummy.flush();

    EXPECT_EQ(expected, responses);
    EXPECT_TRUE(replay.done());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
