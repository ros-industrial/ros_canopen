//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef DRIVER_NODE_HPP_
#define DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"

namespace ros2_canopen
{
/**
 * @brief Canopen Driver Interface
 *
 * This provides an interface that all driver nodes that are loaded
 * by ros2_canopen::DeviceContainer need to implement.
 *
 */
class CanopenDriverInterface
{
public:
  /**
   * @brief Initialise the driver
   *
   * This function will initialise the drivers functionalities.
   * It will be called by the device_container when the node has
   * been added to the executor.
   */
  virtual void init() = 0;

  /**
   * @brief Set the master object
   *
   * This function will set the Canopen Master Objects that are
   * necessary for the driver to be instantiated. It will be called
   * by the device container when the init_driver service is invoked.
   *
   * @param exec
   * @param master
   */
  virtual void set_master(
    std::shared_ptr<lely::ev::Executor> exec,
    std::shared_ptr<lely::canopen::AsyncMaster> master) = 0;

  /**
   * @brief Get the node base interface object
   *
   * This function shall return an rclcpp::node_interfaces::NodeBaseInterface::SharedPtr.
   * The pointer will be used to add the driver node to the executor.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() = 0;

  /**
   * @brief Shutdown the driver
   *
   * This function will shutdown the driver and will especially
   * join all threads to enable a clean shutdown.
   *
   */
  virtual void shutdown() = 0;

  /**
   * @brief Check whether this is a LifecycleNode
   *
   * This function provides runtime information on whether the driver
   * is a lifecycle driver or not.
   *
   * @return true
   * @return false
   */
  virtual bool is_lifecycle() = 0;

  /**
   * @brief Get the node canopen driver interface object
   *
   * This function gives access to the underlying NodeCanopenDriverInterface.
   *
   * @return std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
   */
  virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
  get_node_canopen_driver_interface() = 0;
};

/**
 * @brief Canopen Driver
 *
 * This provides a class, that driver nodes that are based on rclcpp::Node
 * should be derived of. This class implements the ros2_canopen::CanopenDriverInterface.
 *
 */
class CanopenDriver : public CanopenDriverInterface, public rclcpp::Node
{
public:
  std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;
  explicit CanopenDriver(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node("canopen_driver", node_options)
  {
    node_canopen_driver_ = std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp::Node>>(this);
  }

  /**
   * @brief Initialise the driver
   *
   * This function will activate the driver using the instantiated
   * node_canopen_driver_. It will call the init, configure, demand_set_master and activate
   * of the NodeCanopenDriverInterface. If the function finishes without exception,
   * the driver is activated and ready for use.
   *
   */
  virtual void init() override;

  /**
   * @brief Set the master object
   *
   * This function will set the Canopen Master Objects that are
   * necessary for the driver to be instantiated. It will be called
   * by the device container when the init_driver service is invoked.
   *
   * @param exec
   * @param master
   */
  virtual void set_master(
    std::shared_ptr<lely::ev::Executor> exec,
    std::shared_ptr<lely::canopen::AsyncMaster> master) override;

  /**
   * @brief Get the node base interface object
   *
   * This function shall return an rclcpp::node_interfaces::NodeBaseInterface::SharedPtr.
   * The pointer will be used to add the driver node to the executor.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  virtual rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
  {
    return rclcpp::Node::get_node_base_interface();
  }

  /**
   * @brief Shutdown the driver
   *
   * This function will shutdown the driver by calling the
   * shutdown() function of node_canopen_driver_.
   *
   */
  virtual void shutdown() override;

  /**
   * @brief Check whether this is a LifecycleNode
   *
   * @return false
   */
  virtual bool is_lifecycle() override { return false; }

  /**
   * @brief Get the node canopen driver interface object
   *
   * This function gives access to the underlying NodeCanopenDriverInterface.
   *
   * @return std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
   */
  virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
  get_node_canopen_driver_interface()
  {
    return node_canopen_driver_;
  }
};

/**
 * @brief Lifecycle Canopen Driver
 *
 * This provides a class, that driver nodes that are based on rclcpp_lifecycle::LifecycleNode
 * should be derived of. This class implements the ros2_canopen::CanopenDriverInterface.
 *
 */
class LifecycleCanopenDriver : public CanopenDriverInterface, public rclcpp_lifecycle::LifecycleNode
{
protected:
  std::shared_ptr<node_interfaces::NodeCanopenDriverInterface> node_canopen_driver_;

public:
  explicit LifecycleCanopenDriver(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("lifecycle_canopen_driver", node_options)
  {
    node_canopen_driver_ =
      std::make_shared<node_interfaces::NodeCanopenDriver<rclcpp_lifecycle::LifecycleNode>>(this);
  }
  /**
   * @brief Initialise the driver
   *
   * This function will activate the driver using the instantiated
   * node_canopen_driver_. It will call init()
   * of the NodeCanopenDriverInterface. If the function finishes without exception,
   * the driver can be configured.
   *
   */
  virtual void init() override;

  /**
   * @brief Set the master object
   *
   * This function will set the Canopen Master Objects that are
   * necessary for the driver to be instantiated. It will be called
   * by the device container when the init_driver service is invoked.
   *
   * @param exec
   * @param master
   */
  virtual void set_master(
    std::shared_ptr<lely::ev::Executor> exec,
    std::shared_ptr<lely::canopen::AsyncMaster> master) override;

  /**
   * @brief Configure Callback
   *
   * This function will call configure() and demand_set_master() of the
   * node_canopen_driver_.
   *
   * @param state
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state);
  /**
   * @brief Activate Callback
   *
   * This function will call activate() of the
   * node_canopen_driver_.
   *
   * @param state
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate Callback
   *
   * This function will call deactivate() of the
   * node_canopen_driver_.
   *
   * @param state
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate Callback
   *
   * This function will call cleanup() of the
   * node_canopen_driver_.
   *
   * @param state
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate Callback
   *
   * This function will call shutdown() of the
   * node_canopen_driver_.
   *
   * @param state
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the node base interface object
   *
   * This function shall return an rclcpp::node_interfaces::NodeBaseInterface::SharedPtr.
   * The pointer will be used to add the driver node to the executor.
   *
   * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override
  {
    return rclcpp_lifecycle::LifecycleNode::get_node_base_interface();
  }

  /**
   * @brief Shutdown the driver
   *
   * This function will shutdown the driver by calling the
   * shutdown() function of node_canopen_driver_.
   *
   */
  virtual void shutdown() override;
  /**
   * @brief Check whether this is a LifecycleNode
   *
   * @return true
   */
  virtual bool is_lifecycle() override { return true; }
  /**
   * @brief Get the node canopen driver interface object
   *
   * This function gives access to the underlying NodeCanopenDriverInterface.
   *
   * @return std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
   */
  virtual std::shared_ptr<node_interfaces::NodeCanopenDriverInterface>
  get_node_canopen_driver_interface()
  {
    return node_canopen_driver_;
  }
};

}  // namespace ros2_canopen

#endif
