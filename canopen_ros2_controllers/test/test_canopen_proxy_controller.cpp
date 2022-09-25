// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_canopen_proxy_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class CanopenProxyControllerTest : public CanopenProxyControllerFixture<TestableCanopenProxyController>
{
};

// When there are many mandatory parameters, set all by default and remove one by one in a
// parameterized test
TEST_P(CanopenProxyControllerTestParameterizedParameters, one_parameter_is_missing)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);
}

// TODO(anyone): the new gtest version after 1.8.0 uses INSTANTIATE_TEST_SUITE_P
INSTANTIATE_TEST_SUITE_P(
  MissingMandatoryParameterDuringConfiguration, CanopenProxyControllerTestParameterizedParameters,
  ::testing::Values(
    std::make_tuple(std::string("joint"), rclcpp::ParameterValue(std::string({})))));

TEST_F(CanopenProxyControllerTest, joint_names_parameter_not_set)
{
  SetUpController(false);

  ASSERT_TRUE(controller_->joint_name_.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_ERROR);

  ASSERT_TRUE(controller_->joint_name_.empty());
}

TEST_F(CanopenProxyControllerTest, all_parameters_set_configure_success)
{
  SetUpController();

  ASSERT_TRUE(controller_->joint_name_.empty());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_THAT(controller_->joint_name_, joint_name_);
}

TEST_F(CanopenProxyControllerTest, check_exported_intefaces)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);

  auto command_intefaces = controller_->command_interface_configuration();
  ASSERT_EQ(command_intefaces.names.size(), joint_command_values_.size());
  for (size_t i = 0; i < command_intefaces.names.size(); ++i) {
    EXPECT_EQ(command_intefaces.names[i], joint_name_ + "/" + command_interface_names_[i]);
  }

  auto state_intefaces = controller_->state_interface_configuration();
  ASSERT_EQ(state_intefaces.names.size(), joint_state_values_.size());
  for (size_t i = 0; i < state_intefaces.names.size(); ++i) {
    EXPECT_EQ(state_intefaces.names[i], joint_name_ + "/" + state_interface_names_[i]);
  }
}

TEST_F(CanopenProxyControllerTest, activate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // check that the message is reset
  auto msg = controller_->input_cmd_.readFromNonRT();
  EXPECT_EQ((*msg)->index, 0u);
  EXPECT_EQ((*msg)->subindex, 0u);
  EXPECT_EQ((*msg)->type, 0u);
  EXPECT_EQ((*msg)->data, 0u);

}

TEST_F(CanopenProxyControllerTest, update_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(CanopenProxyControllerTest, deactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CanopenProxyControllerTest, reactivate_success)
{
  SetUpController();

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->command_interfaces_[CommandInterfaces::TPDO_DATA].get_value(), 101.101);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CommandInterfaces::TPDO_DATA].get_value()));
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_TRUE(std::isnan(controller_->command_interfaces_[CommandInterfaces::TPDO_DATA].get_value()));

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(CanopenProxyControllerTest, test_sequence_configure_activate)
{
  SetUpController();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);
}

TEST_F(CanopenProxyControllerTest, test_update_logic_fast)
{
  SetUpController();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(controller_->get_node()->get_node_base_interface());
  executor.add_node(service_caller_node_->get_node_base_interface());

  ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), NODE_SUCCESS);
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), NODE_SUCCESS);

  // set command statically as value for good type
  static constexpr uint32_t TEST_GOOD_TYPE = 8;
  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  msg->index = 0u;
  msg->subindex = 0u;
  msg->type = TEST_GOOD_TYPE;
  msg->data = 0u;

  controller_->input_cmd_.writeFromNonRT(msg);

  ASSERT_EQ(
    controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);

  EXPECT_EQ(
    static_cast<uint32_t>(controller_->command_interfaces_[CommandInterfaces::TPDO_TYPE].get_value()),
    TEST_GOOD_TYPE);
}
