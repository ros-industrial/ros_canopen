#include <chrono>
#include "canopen_core/driver_error.hpp"
#include "canopen_core/master_error.hpp"
#include "canopen_core/device_container_error.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace testing;

TEST(DriverErrorTest, test_what)
{
    DriverException test_obj("hello");
    EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}

TEST(MasterErrorTest, test_what)
{
    MasterException test_obj("hello");
    EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}

TEST(DeviceContainerErrorTest, test_what)
{
    DeviceContainerException test_obj("hello");
    EXPECT_TRUE(std::string("hello").compare(test_obj.what()) == 0);
}