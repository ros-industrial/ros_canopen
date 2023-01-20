Architecture
=============

The architecture of the ROS2 CANopen stack is based on the composition
concept of ROS2. In ROS2 components are dynamically loadable ROS2 nodes
which are loaded by a component manager.

Device Container
""""""""""""""""""
The core of the ROS2 CANopen stack is the ros2_canopen::DeviceContainer which implements
the rclcpp::ComponentManager class. The DeviceContainer enables loading drivers for the
CANopen master and the devices on the bus, that have been exported as rclcpp_components.
CANopen Master drivers need to implement the ros2_canopen::CanopenMasterInterface, CANopen
Device drivers need to implement the ros2_canopen::CanopenDriverInterface.

The difference between the ros2_canopen::DeviceContainer and the rclcpp::ComponentContainer
is that the device container loads only master and driver components specified in the bus
configuration (bus.yml). It does not have services for loading components online. All components
that are connected or will be connected to the CANopen Bus need to be known and specified in
the bus.yml before starting the device container.

.. uml::

  rclcpp::ComponentManager <|-- ros2_canopen::DeviceContainer

  class rclcpp::ComponentManager
  {
    std::weak_ptr<rclcpp::Executor> executor_
    std::vector<ComponentResource> get_component_resources(std::string package_name)
    std::shared_ptr<NodeFactory> create_component_factory(ComponentResource resource)
    void on_list_nodes(...)
    virtual void set_executor(const std::weak_ptr<rclcpp::Executor> executor)
  }


  class ros2_canopen::DeviceContainer
  {
    std::shared_ptr<CanopenMasterInterface> can_master_
    std::shared_ptr<LifecycleManager> lifecycle_manager_
    std::map<uint16_t, std::shared_ptr<CanopenDriverInterface>> drivers_
    void on_list_nodes(...) override
    virtual void set_executor(const std::weak_ptr<rclcpp::Executor> executor) override
  }

The device container will first load the master specified in the bus configuration. Then
it will load the drivers specified in the bus configuration. If the master and drivers
specified in the bus configuration are managed nodes it will as well load the ros2_canopen::LifecycleManager.


CANopen Master Driver Architecture
"""""""""""""""""""""""""""""""""""

The architecture for CANopen master drivers looks as depicted in the class diagram. All master drivers
consist of three main classes.

The first class is the functionality class that contains the drivers functionailities independently of
the ROS2 node type. This class needs to implement the ros2_canopen::node_interfaces::NodeCanopenMasterInterface.
ros2_canopen::node_interfaces::NodeCanopenMaster is an abstract class that provides some useful functionality and
implements the ros2_canopen::node_interfaces::NodeCanopenMasterInterface. Usually, master drivers will inherit from
ros2_canopen::node_interfaces::NodeCanopenMaster.

The second class is the class that wraps the functionality class in a rclcpp::Node. This class should implement the
ros2_canopen::CanopenMasterInterface. The canopen_core package provides a convenience class ros2_canopen::CanopenMaster
that should be inherited from and implements the ros2_canopen::CanopenMasterInterface.

The third class is the class that wraps the functionality class in a rclcpp_lifecycle::LifecycleNode. This class should implement the
ros2_canopen::CanopenMasterInterface. The canopen_core package provides a convenience class ros2_canopen::LifecycleCanopenMaster
that should be inherited from and implements the ros2_canopen::CanopenMasterInterface.

.. uml::


  package "canopen_core" {
    interface ros2_canopen::CanopenMasterInterface
    interface ros2_canopen::node_interfaces::NodeCanopenMasterInterface
    abstract ros2_canopen::LifecycleCanopenMaster
    abstract ros2_canopen::node_interfaces::NodeCanopenMaster
    abstract ros2_canopen::CanopenMaster


    ros2_canopen::node_interfaces::NodeCanopenMasterInterface - ros2_canopen::CanopenMaster : < has
    ros2_canopen::LifecycleCanopenMaster - ros2_canopen::node_interfaces::NodeCanopenMasterInterface : > has

    ros2_canopen::CanopenMasterInterface <|-- ros2_canopen::LifecycleCanopenMaster
    ros2_canopen::node_interfaces::NodeCanopenMasterInterface <|-- ros2_canopen::node_interfaces::NodeCanopenMaster
    ros2_canopen::CanopenMasterInterface <|-- ros2_canopen::CanopenMaster

  }

  package "canopen_master_driver" {

    class ros2_canopen::LifecycleMasterDriver << (C, blue) Component>>
    class ros2_canopen::node_interfaces::NodeCanopenBasicMaster
    class ros2_canopen::MasterDriver << (C, blue) Component>>
    ros2_canopen::LifecycleMasterDriver - ros2_canopen::node_interfaces::NodeCanopenBasicMaster: > has
    ros2_canopen::node_interfaces::NodeCanopenBasicMaster - ros2_canopen::MasterDriver : < has
    ros2_canopen::LifecycleCanopenMaster <|-- ros2_canopen::LifecycleMasterDriver
    ros2_canopen::node_interfaces::NodeCanopenMaster <|-- ros2_canopen::node_interfaces::NodeCanopenBasicMaster
    ros2_canopen::CanopenMaster <|-- ros2_canopen::MasterDriver
  }


CANopen Device Driver Architecture
"""""""""""""""""""""""""""""""""""

The architecture for CANopen device drivers looks as depicted in the class diagram. All device drivers
consist of three main classes.

The first class is the functionality class that contains the drivers functionailities independently of
the ROS2 node type. This class needs to implement the ros2_canopen::node_interfaces::NodeCanopenDriverInterface.
ros2_canopen::node_interfaces::NodeCanopenDriver is an abstract class that provides some useful functionality and
implements the ros2_canopen::node_interfaces::NodeCanopenDriverInterface. If you plan to write a driver from scratch
based on Lely Core library, your functionality class should inherit from ros2_canopen::node_interfaces::NodeCanopenDriver.
If you want to use the existing lely_driver_bridge, your functionality class should inherit from ros2_canopen::NodeCanopenBaseDriver.

The second class is the class that wraps the functionality class in a rclcpp::Node. This class should implement the
ros2_canopen::CanopenDriverInterface. The canopen_core package provides a convenience class ros2_canopen::CanopenDriver
that should be inherited from and implements the ros2_canopen::CanopenDriverInterface.

The third class is the class that wraps the functionality class in a rclcpp_lifecycle::LifecycleNode. This class should implement the
ros2_canopen::CanopenDriverInterface. The canopen_core package provides a convenience class ros2_canopen::LifecycleCanopenDriver
that should be inherited from and implements the ros2_canopen::CanopenDriverInterface.

.. uml::

  package "canopen_core" {
    interface ros2_canopen::CanopenDriverInterface
    interface ros2_canopen::node_interfaces::NodeCanopenDriverInterface
    abstract ros2_canopen::LifecycleCanopenDriver
    abstract ros2_canopen::node_interfaces::NodeCanopenDriver
    abstract ros2_canopen::CanopenDriver


    ros2_canopen::node_interfaces::NodeCanopenDriverInterface - ros2_canopen::CanopenDriver : < has
    ros2_canopen::LifecycleCanopenDriver - ros2_canopen::node_interfaces::NodeCanopenDriverInterface : > has

    ros2_canopen::CanopenDriverInterface <|-- ros2_canopen::LifecycleCanopenDriver
    ros2_canopen::node_interfaces::NodeCanopenDriverInterface <|-- ros2_canopen::node_interfaces::NodeCanopenDriver
    ros2_canopen::CanopenDriverInterface <|-- ros2_canopen::CanopenDriver

  }


  package "canopen_base_driver" {

    class ros2_canopen::LifecycleBaseDriver << (C, blue) Component>>
    class ros2_canopen::node_interfaces::NodeCanopenBaseDriver
    class ros2_canopen::BaseDriver << (C, blue) Component>>
    ros2_canopen::LifecycleBaseDriver - ros2_canopen::node_interfaces::NodeCanopenBaseDriver: > has
    ros2_canopen::node_interfaces::NodeCanopenBaseDriver - ros2_canopen::BaseDriver : < has
    ros2_canopen::LifecycleCanopenDriver <|-- ros2_canopen::LifecycleBaseDriver
    ros2_canopen::node_interfaces::NodeCanopenDriver <|-- ros2_canopen::node_interfaces::NodeCanopenBaseDriver
    ros2_canopen::CanopenDriver <|-- ros2_canopen::BaseDriver
  }

  package "canopen_proxy_driver" {
    class ros2_canopen::LifecycleProxyDriver << (C, blue) Component>>
    class ros2_canopen::node_interfaces::NodeCanopenProxyDriver
    class ros2_canopen::ProxyDriver << (C, blue) Component>>
    ros2_canopen::LifecycleProxyDriver - ros2_canopen::node_interfaces::NodeCanopenProxyDriver: > has
    ros2_canopen::node_interfaces::NodeCanopenProxyDriver - ros2_canopen::ProxyDriver : < has
    ros2_canopen::LifecycleCanopenDriver <|-- ros2_canopen::LifecycleProxyDriver
    ros2_canopen::node_interfaces::NodeCanopenBaseDriver <|-- ros2_canopen::node_interfaces::NodeCanopenProxyDriver
    ros2_canopen::CanopenDriver <|-- ros2_canopen::ProxyDriver
  }

  package "canopen_402_driver" {
    class ros2_canopen::LifecycleCia402Driver << (C, blue) Component>>
    class ros2_canopen::node_interfaces::NodeCanopen402Driver
    class ros2_canopen::Cia402Driver << (C, blue) Component>>
    ros2_canopen::LifecycleCia402Driver - ros2_canopen::node_interfaces::NodeCanopen402Driver: > has
    ros2_canopen::node_interfaces::NodeCanopen402Driver - ros2_canopen::Cia402Driver : < has
    ros2_canopen::LifecycleCanopenDriver <|-- ros2_canopen::LifecycleCia402Driver
    ros2_canopen::node_interfaces::NodeCanopenProxyDriver <|-- ros2_canopen::node_interfaces::NodeCanopen402Driver
    ros2_canopen::CanopenDriver <|-- ros2_canopen::Cia402Driver
  }
