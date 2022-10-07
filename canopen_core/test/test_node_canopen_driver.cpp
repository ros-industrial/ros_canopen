#include "canopen_core/driver_node.hpp"
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include "gtest/gtest.h"
#include "gmock/gmock.h"

using namespace ros2_canopen;
using namespace ros2_canopen::node_interfaces;
using namespace testing;
template<class NODETYPE>
class MockNodeCanopenDriver : public NodeCanopenDriver<NODETYPE>
{
public:
    FRIEND_TEST(NodeCanopenDriverTest, test_construct);
	FRIEND_TEST(NodeCanopenDriverTest, test_init);
	FRIEND_TEST(NodeCanopenDriverTest, test_init_fail_configured);
	FRIEND_TEST(NodeCanopenDriverTest, test_init_fail_activated);
	FRIEND_TEST(NodeCanopenDriverTest, test_set_master);
	FRIEND_TEST(NodeCanopenDriverTest, test_configure);
	FRIEND_TEST(NodeCanopenDriverTest, test_configure_fail_not_initialsed);
	FRIEND_TEST(NodeCanopenDriverTest, test_configure_fail_configured);
	FRIEND_TEST(NodeCanopenDriverTest, test_configure_fail_activated);
	FRIEND_TEST(NodeCanopenDriverTest, test_activate);
	FRIEND_TEST(NodeCanopenDriverTest, test_activate_failures);
	FRIEND_TEST(NodeCanopenDriverTest, test_deactivate);
	FRIEND_TEST(NodeCanopenDriverTest, test_deactivate_failures);
	FRIEND_TEST(NodeCanopenDriverTest, test_cleanup);
	FRIEND_TEST(NodeCanopenDriverTest, test_cleanup_failures);
	FRIEND_TEST(NodeCanopenDriverTest, test_shutdown);

    MockNodeCanopenDriver(rclcpp::Node* node)
    : NodeCanopenDriver<NODETYPE>(node)
    {
    }
	MOCK_METHOD0_T(add_to_master, void());
	MOCK_METHOD0_T(remove_from_master, void());

    void DelegateToBase()
    {
        // ON_CALL(*this, load_component)
        //     .WillByDefault(
        //         [this](
        //             std::string & package_name, std::string & driver_name, uint16_t node_id, std::string & node_name,
        //             std::vector<rclcpp::Parameter> & params) -> bool
        //         { return DeviceContainer::load_component(package_name, driver_name, node_id, node_name, params); });
    }
};

class NodeCanopenDriverTest : public testing::Test
{
public:
    std::shared_ptr<MockNodeCanopenDriver<rclcpp::Node>> node_canopen_driver;
	rclcpp::Node::SharedPtr node;
    void SetUp() override
    {
        RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "SetUp");
        rclcpp::init(0, nullptr);
		node = std::make_shared<rclcpp::Node>("driver");
        node_canopen_driver = std::make_shared<MockNodeCanopenDriver<rclcpp::Node>>(node.get());
        node_canopen_driver->DelegateToBase();
    }

    void TearDown() override
    {
        RCLCPP_INFO(rclcpp::get_logger("CanopenMasterTest"), "TearDown");
        rclcpp::shutdown();
    }
};

TEST_F(NodeCanopenDriverTest, test_construct)
{
	EXPECT_EQ(node.get(), node_canopen_driver->node_);
}

TEST_F(NodeCanopenDriverTest, test_init)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	node_canopen_driver->init();
	EXPECT_TRUE(node_canopen_driver->initialised_.load());
	EXPECT_TRUE(node->has_parameter("container_name"));
	EXPECT_TRUE(node->has_parameter("node_id"));
	EXPECT_TRUE(node->has_parameter("non_transmit_timeout"));
	EXPECT_TRUE(node->has_parameter("config"));
}

TEST_F(NodeCanopenDriverTest, test_init_fail_configured)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->init(), DriverException);
	EXPECT_FALSE(node_canopen_driver->initialised_.load());
	EXPECT_FALSE(node->has_parameter("container_name"));
	EXPECT_FALSE(node->has_parameter("node_id"));
	EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
	EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenDriverTest, test_init_fail_activated)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->init(), DriverException);
	EXPECT_FALSE(node_canopen_driver->initialised_.load());
	EXPECT_FALSE(node->has_parameter("container_name"));
	EXPECT_FALSE(node->has_parameter("node_id"));
	EXPECT_FALSE(node->has_parameter("non_transmit_timeout"));
	EXPECT_FALSE(node->has_parameter("config"));
}

TEST_F(NodeCanopenDriverTest, test_set_master)
{
	std::shared_ptr<lely::ev::Executor> exec;
	std::shared_ptr<lely::canopen::AsyncMaster> master;
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);

	node_canopen_driver->set_master(exec, master);
	EXPECT_TRUE(node_canopen_driver->master_set_.load());

	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);

	EXPECT_THROW(node_canopen_driver->set_master(exec, master), DriverException);

	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);

	EXPECT_THROW(node_canopen_driver->set_master(exec, master), DriverException);
}

TEST_F(NodeCanopenDriverTest, test_configure)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	node_canopen_driver->init();
	node->set_parameter(rclcpp::Parameter("config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
	node_canopen_driver->configure();
	EXPECT_TRUE(node_canopen_driver->configured_.load());
}

TEST_F(NodeCanopenDriverTest, test_configure_fail_not_initialsed)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->configure(), DriverException);
	EXPECT_FALSE(node_canopen_driver->configured_.load());
}

TEST_F(NodeCanopenDriverTest, test_configure_fail_configured)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	node_canopen_driver->init();
	node_canopen_driver->configured_.store(true);
	node->set_parameter(rclcpp::Parameter("config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
	EXPECT_THROW(node_canopen_driver->configure(), DriverException);
}

TEST_F(NodeCanopenDriverTest, test_configure_fail_activated)
{
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	node_canopen_driver->init();
	node_canopen_driver->activated_.store(true);
	node->set_parameter(rclcpp::Parameter("config", "node_id: 1\ndriver: \"ros2_canopen::CanopenDriver\"\npackage:\"canopen_core\"\n"));
	EXPECT_THROW(node_canopen_driver->configure(), DriverException);
	EXPECT_FALSE(node_canopen_driver->configured_.load());
}

TEST_F(NodeCanopenDriverTest, test_activate)
{
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_CALL(*node_canopen_driver, add_to_master()).Times(1).WillOnce(Return());
	node_canopen_driver->activate();
	EXPECT_TRUE(node_canopen_driver->activated_.load());
}

TEST_F(NodeCanopenDriverTest, test_activate_failures)
{
	node_canopen_driver->initialised_.store(false);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->activate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(false);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->activate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->activate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->activate(), DriverException);
}

TEST_F(NodeCanopenDriverTest, test_deactivate)
{
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_CALL(*node_canopen_driver, remove_from_master()).Times(1).WillOnce(Return());
	node_canopen_driver->deactivate();
	EXPECT_FALSE(node_canopen_driver->activated_.load());
}

TEST_F(NodeCanopenDriverTest, test_deactivate_failures)
{
	node_canopen_driver->initialised_.store(false);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->deactivate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(false);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->deactivate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->deactivate(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->deactivate(), DriverException);
}

TEST_F(NodeCanopenDriverTest, test_cleanup)
{
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	node_canopen_driver->cleanup();
	EXPECT_FALSE(node_canopen_driver->configured_.load());
}

TEST_F(NodeCanopenDriverTest, test_cleanup_failures)
{
	node_canopen_driver->initialised_.store(false);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->cleanup(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	EXPECT_THROW(node_canopen_driver->cleanup(), DriverException);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_THROW(node_canopen_driver->cleanup(), DriverException);
}

TEST_F(NodeCanopenDriverTest, test_shutdown)
{
	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(true);
	EXPECT_CALL(*node_canopen_driver, remove_from_master()).Times(1).WillOnce(Return());
	EXPECT_NO_THROW(node_canopen_driver->shutdown());
	EXPECT_FALSE(node_canopen_driver->master_set_.load());
	EXPECT_FALSE(node_canopen_driver->initialised_.load());
	EXPECT_FALSE(node_canopen_driver->configured_.load());
	EXPECT_FALSE(node_canopen_driver->activated_.load());

	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(true);
	node_canopen_driver->activated_.store(false);
	EXPECT_NO_THROW(node_canopen_driver->shutdown());
	EXPECT_FALSE(node_canopen_driver->master_set_.load());
	EXPECT_FALSE(node_canopen_driver->initialised_.load());
	EXPECT_FALSE(node_canopen_driver->configured_.load());
	EXPECT_FALSE(node_canopen_driver->activated_.load());

	node_canopen_driver->master_set_.store(true);
	node_canopen_driver->initialised_.store(true);
	node_canopen_driver->configured_.store(false);
	node_canopen_driver->activated_.store(false);
	EXPECT_NO_THROW(node_canopen_driver->shutdown());
	EXPECT_FALSE(node_canopen_driver->master_set_.load());
	EXPECT_FALSE(node_canopen_driver->initialised_.load());
	EXPECT_FALSE(node_canopen_driver->configured_.load());
	EXPECT_FALSE(node_canopen_driver->activated_.load());
}