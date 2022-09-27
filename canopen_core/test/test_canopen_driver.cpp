#include "canopen_core/driver_node.hpp"
#include "gtest/gtest.h"
#include <lely/ev/loop.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/timer.hpp>

class RclCppFixture
{
public:
    RclCppFixture()
    {
        rclcpp::init(0, nullptr);
    }
    ~RclCppFixture()
    {
        rclcpp::shutdown();
    }
};
RclCppFixture g_rclcppfixture;

TEST(CanopenDriver, test_canopen_driver_instantiation)
{
    ros2_canopen::CanopenDriver driver;
}

TEST(CanopenDriver, test_lifecycle_canopen_driver_instantiation)
{
    ros2_canopen::LifecycleCanopenDriver driver;
}