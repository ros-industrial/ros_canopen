// Bring in my package's API, which is what I'm testing
#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/dummy.h>

// Bring in gtest
#include <gtest/gtest.h>

class DummyInterfaceTest : public ::testing::Test{
public:
    std::list<std::string> responses;
    can::ThreadedDummyInterfaceSharedPtr dummy;
    DummyInterfaceTest() : dummy(std::make_shared<can::ThreadedDummyInterface>()), listener(dummy->createMsgListenerM(this, &DummyInterfaceTest::handle)) {}

    void handle(const can::Frame &f){
        responses.push_back(can::tostring(f, true));
    }
    can::FrameListenerConstSharedPtr listener;
};

// Declare a test
TEST_F(DummyInterfaceTest, testCase1)
{
    can::DummyBus bus("testCase1");
    dummy->add("0#8200", "701#00", false);
    dummy->init(bus.name, true, can::NoSettings::create());

    std::list<std::string> expected;

    dummy->send(can::toframe("0#8200"));
    dummy->flush();
   
    expected.push_back("0#8200");
    expected.push_back("701#00");

    EXPECT_EQ(expected, responses);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
