Creating a new device driver
============================

Creating your own device driver is fairly easy in ros2_canopen. You should do this if you
need to create a driver for a specific device or a specific device profile that is not yet supported. If you create
a driver for a device profile we are happy to integrate the package into this repository - simply create
a PR.

What you need to do
""""""""""""""""""""
To create a new driver you need to implement at least two classes. One being the functional class,
that contains your drivers functionalities. The other being a ROS2 wrapper node. Generally we recommend
creating one ROS2 wrapper node and another ROS2 lifecycle wrapper node.

How you do it
""""""""""""""
First you need to decide from which extension point you want to start. Usually, this is either the core interface, the base driver
or the proxy driver. Base driver provides you with all necessary callbacks for CANopen functionalities but does
not come with any ROS2 interface. Proxy driver has a simple forwarding ROS2 interface that is useful for any driver.
The core interface comes without any functionality, you need to implement everything on your own.
We recommend creating you driver based on Proxy driver, this will be explained here.

Create the package
------------------
Create your new package using the standard ros2 pkg commands. Make sure you add the following dependencies:

* rclcpp
* rclcpp_components
* rclcpp_lifecycle
* lifecycle_msgs
* canopen_core
* canopen_interfaces
* canopen_base_driver
* canopen_proxy_driver
* lely_core_libraries
* std_msgs
* std_srvs

Once done add a subfolder ``node_interfaces`` in the ``src/`` and the ``include/[pacakge_name]/`` folders.


Create the functionality class
------------------------------
The functionality class is structured similar to a LifecycleNode. The functionality class
has the following callback functions that are related to lifecycle which you can override:

* void configure(bool called_from_base)
* void activate(bool called_from_base)
* void deactivate(bool called_from_base)
* void cleanup(bool called_from_base)
* void shutdown(bool called_from_base)

CANopen functionality
*********************
In addition to the functions there are callbacks for CANopen functionality that you can
override:

* void on_rpdo(COData data)
* void on_nmt(NmtState nmt_state)

To interact with the CANopen device you can use the ros2_canopen::LelyDriverBridge object,
that is stored in the functionality class (this->driver_). The ros2_canopen::LelyDriverBridge
provides the following functions you should use in your driver:

* void nmt_command(NmtState nmt_state)
* void tpdo_transmit(COData data)
* std::future<bool>async_sdo_write(COData data)
* std::future<COData>async_sdo_read(COData data)

.. note::

   The CANopen related functionality can only be used in the activate function or timers/threads that
   were started by the activate function. If you start timers or threads in the activate function, that
   use CANopen functionality, these have to be stopped in the deactivate function.

ROS2 functionality
******************
ROS2 functionlity is available via the ``node_`` object of the functionality class. This
object has a templated type and can either be a ``rclcpp::Node`` or ``rclcpp_lifecycle::LifecycleNode``.
You can use the standard functions like create_timer, create_publisher etc.

.. note::

   Currently it seems, that when you use template functions i.e. ``node_->create_publisher<xyz>(...)`` you
   need to create a template specialisation.



Your class declaration should look like this:

.. code-block:: C++
   :name: "node_interfaces/node_canopen_xxx_driver.hpp"

   node_interfaces/node_canopen_xxx_driver.hpp:

   #include <canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp>

   template <class NODETYPE>
   class NodeCanopenXXXDriver : public NodeCanopenProxyDriver<NODETYPE>
   {
      static_assert(
            std::is_base_of<rclcpp::Node, NODETYPE>::value ||
               std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
            "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");
   public:
      NodeCanopenXXXDriver(NODETYPE *node);

      /*Your overrides etc. go here*/
   };

Your member definitions go here:

.. code-block:: C++
   :name: "node_interfaces/node_canopen_xxx_driver_impl.hpp"

   node_interfaces/node_canopen_xxx_driver.hpp:

   #include node_interfaces/node_canopen_xxx_driver.hpp

   /*Your function definitions go here.*/

Your explicit template instantiations go here:

.. code-block:: C++
   :name: "node_interfaces/node_canopen_xxx_driver.cpp"

   node_interfaces/node_canopen_xxx_driver.cpp:

   #include node_interfaces/node_canopen_xxx_driver.hpp
   #include node_interfaces/node_canopen_xxx_driver_impl.hpp

   template class <ros2_canopen>::node_interfaces::NodeCanopenXXXDriver<rclcpp::Node>;
   template class <ros2_canopen>::node_interfaces::NodeCanopenXXXDriver<rclcpp_lifecycle::LifecycleNode>;


Create the ROS2 wrapper classes
-------------------------------

The ROS2 wrapper classes are fairly easy to create once you wrote the functionality
class. The wrappers simply use the functionality class to provide the functionality.
The ROS2 wrapper class should always be derived from ``ros2_canopen::CanopenDriver`` or
``ros2_canopen::LifecycleCanopenDriver`` .


The declaration should look like this:

.. code::

   lifecycle_xxx_driver.hpp:

   #include "canopen_xxx_driver/node_interfaces/node_canopen_xxx_driver.hpp"
   #include "canopen_core/driver_node.hpp"

   /**
      * @brief Lifecycle Proxy Driver
      *
      * A very basic driver without any functionality.
      *
      */
   class LifecycleXXXDriver : public ros2_canopen::LifecycleCanopenDriver
   {
      std::shared_ptr<node_interfaces::NodeCanopenXXXDriver<rclcpp_lifecycle::LifecycleNode>> node_canopen_xxx_driver_;
   public:
      LifecycleXXXDriver(rclcpp::NodeOptions node_options = rclcpp::NodeOptions());
   };

The definitions should look like this:

.. code::


   #include "canopen_xxx_driver/lifecycle_proxy_driver.hpp"

   using namespace ros2_canopen;


   LifecycleXXXDriver::LifecycleXXXDriver(rclcpp::NodeOptions node_options) : LifecycleCanopenDriver(node_options)
   {
   node_canopen_xxx_driver_ = std::make_shared<node_interfaces::NodeCanopenXXXDriver<rclcpp_lifecycle::LifecycleNode>>(this);
   node_canopen_proxy_driver_ = std::static_pointer_cast<node_interfaces::NodeCanopenProxyDriver>(node_canopen_xxx_driver_);
   node_canopen_driver_ = std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(node_canopen_xxx_driver_);
   }

   #include "rclcpp_components/register_node_macro.hpp"
   RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleXXXDriver)


Adapt the CMakeLists.txt
************************
The CMakeLists.txt file should look like this:

.. code:: CMAKE

   cmake_minimum_required(VERSION 3.8)
   project(canopen_xxx_driver)

   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
   add_compile_options(-Wall -Wpedantic -Wextra -Wno-unused-parameter)
   endif()

   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(ament_cmake_ros REQUIRED)
   find_package(rclcpp REQUIRED)
   find_package(rclcpp_lifecycle REQUIRED)
   find_package(rclcpp_components REQUIRED)
   find_package(canopen_core REQUIRED)
   find_package(canopen_interfaces REQUIRED)
   find_package(canopen_base_driver REQUIRED)
   find_package(canopen_proxy_driver REQUIRED)
   find_package(lely_core_libraries REQUIRED)
   find_package(std_msgs REQUIRED)
   find_package(std_srvs REQUIRED)

   set(dependencies
   rclcpp
   rclcpp_components
   rclcpp_lifecycle
   lifecycle_msgs
   canopen_core
   canopen_interfaces
   canopen_base_driver
   canopen_proxy_driver
   lely_core_libraries
   std_msgs
   std_srvs
   )

   # Functionality library
   add_library(node_canopen_xxx_driver
   src/node_interfaces/node_canopen_xxx_driver.cpp
   )
   target_compile_features(node_canopen_xxx_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
   target_compile_options(node_canopen_xxx_driver PUBLIC -Wl,--no-undefined)
   target_include_directories(node_canopen_xxx_driver PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

   ament_target_dependencies(
   node_canopen_xxx_driver
   ${dependencies}
   )

   # Lifecycle driver
   add_library(lifecycle_xxx_driver
   src/lifecycle_xxx_driver.cpp
   )
   target_compile_features(lifecycle_xxx_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
   target_compile_options(lifecycle_xxx_driver PUBLIC -Wl,--no-undefined)
   target_include_directories(lifecycle_xxx_driver PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

   target_link_libraries(lifecycle_xxx_driver
   node_canopen_xxx_driver
   )
   ament_target_dependencies(
   lifecycle_xxx_driver
   ${dependencies}
   )
   # Causes the visibility macros to use dllexport rather than dllimport,
   # which is appropriate when building the dll but not consuming it.
   target_compile_definitions(lifecycle_xxx_driver PRIVATE "CANOPEN_XXX_DRIVER_BUILDING_LIBRARY")

   rclcpp_components_register_nodes(lifecycle_xxx_driver "ros2_canopen::LifecycleXXXDriver")
   set(node_plugins "${node_plugins}ros2_canopen::LifecycleXXXDriver;$<TARGET_FILE:lifecycle_xxx_driver >\n")


   # Non lifecycle driver
   add_library(xxx_driver
   src/xxx_driver.cpp
   )
   target_compile_features(xxx_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
   target_compile_options(xxx_driver PUBLIC -Wl,--no-undefined)
   target_include_directories(xxx_driver PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)
   target_link_libraries(xxx_driver
   node_canopen_xxx_driver
   )

   ament_target_dependencies(
   xxx_driver
   ${dependencies}
   )

   # Causes the visibility macros to use dllexport rather than dllimport,
   # which is appropriate when building the dll but not consuming it.
   target_compile_definitions(xxx_driver PRIVATE "CANOPEN_XXX_DRIVER_BUILDING_LIBRARY")

   rclcpp_components_register_nodes(xxx_driver "ros2_canopen::XXXDriver")
   set(node_plugins "${node_plugins}ros2_canopen::XXXDriver;$<TARGET_FILE:xxx_driver >\n")

   install(
   DIRECTORY include/
   DESTINATION include
   )

   install(
   TARGETS lifecycle_xxx_driver xxx_driver node_canopen_xxx_driver
   EXPORT export_${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
   )

   if(BUILD_TESTING)
   endif()

   ament_export_include_directories(
   include
   )
   ament_export_libraries(
   lifecycle_xxx_driver
   xxx_driver
   node_canopen_xxx_driver
   )
   ament_export_targets(
   export_${PROJECT_NAME}
   )

   ament_package()
