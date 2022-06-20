Creating a Configuration Package
========================================
In order to use the ros2_canopen stack for your robot, you need to
create a configuration package, that holds the configuration of the
bus as well as you launch script. The following sections detail the
steps to create such a package.


Package creation
------------------------------
We need to create a package with dependencies for ros2_canopen and lely_core_libraries.

.. code-block:: console

  $ ros2 pkg create --dependencies ros2_canopen lely_core_libraries --build-type ament_cmake <package_name>
  $ cd <package_name>
  $ rm -rf inlcude src
  $ mkdir config
  $ mkdir launch

Bus configuration creation
------------------------------

1. **Gather required information**
    Gather all EDS files of the devices connected to the bus and store them
    in your configuration package in the conf/ directory.

2. **Writing your bus.yml file** 
    First create the configuration yml file in the conf folder.
    
    .. code-block:: console

      $ touch bus.yml

    Open the file in the editor of your choice and create the master description.
    
    .. code-block:: yaml

      master:
        node_id: [node id]
        package: [ros2 package where to find the master driver] 
        driver: [qualified name of the master driver]
    
    And add other configuration data as necessary. A documentation of configuration options
    available can be found in the :doc:`configuration` documentation.

    Once you have defined the configuration of your master, add your slaves. The following
    describes the mandatory data per slave. Further configuration options can be found in the :doc:`configuration` documentation.
    The slave name is the node name that will be assigned to the driver.

    .. code-block:: yaml

      [unique slave name]:
        node_id: [node id]
        package: [ros2 package where to find the driver] 
        driver: [qualified name of the driver]
        enable_lazy_load: false


Launch configuration creation
-----------------------------

Create a launch folder in your package directory and a launch file.

.. code-block:: console

  mkdir launch
  touch bring_up.launch.py

Add the following code and adjust to your needs:

.. code-block:: python

  def generate_launch_description():
        """Generate launch description with multiple components."""
        path_file = os.path.dirname(__file__)

        ld = launch.LaunchDescription()

        master_node = launch_ros.actions.Node(
            name="device_manager_node",
            namespace="", 
            package="canopen_core", 
            output="screen", 
            executable="device_manager_node",
            parameters= [{
                "bus_config": os.path.join(path_file, ".." ,  "config" , "bus.yml"),
                "master_config": os.path.join(path_file, ".." , "config" , "master.dcf"),
                "master_bin": os.path.join(path_file, ".." , "config" , "master.bin"),
                "can_interface_name": "can0"
                }
            ],
        )

        ld.add_action(master_node)

        return ld

By setting parameter enable_lazy_load to false, all drivers will be loaded on start-up.


CMAKE Configuration creation
-----------------------------
We want colcon to install launch and configuration files that are stored
in launch and config folder.

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.8)
  project(trinamic_pd42_can)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(ros2_canopen REQUIRED)
  find_package(lely_core_libraries REQUIRED)

  # generate master dcf
  dcfgen(${CMAKE_CURRENT_SOURCE_DIR}/config/ bus.yml ${CMAKE_BINARY_DIR}/config/)

  # install launch file
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )

  # install configuration files
  install(DIRECTORY
    config/
    DESTINATION share/${PROJECT_NAME}/config/
  )
  install(
    DIRECTORY ${CMAKE_BINARY_DIR}/config/
    DESTINATION share/${PROJECT_NAME}/config/
  )


  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
  endif()

  ament_package()







