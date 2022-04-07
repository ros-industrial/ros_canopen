Creating a Configuration Package
========================================

ROS2 package creation
------------------------------

1. **Create a configuration package**
    .. code-block:: console

      $ ros2 pkg create --dependencies ros2_canopen --build-type ament_cmake <package_name>
      $ cd <package_name>
      $ rm -rf inlcude src
      $ mkdir config
      $ mkdir launch

2. **Adjust your CMAKELists.txt file**
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


      # install launch file
      install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
      )

      # install configuration file
      install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/
      )


      if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
      endif()

      ament_package()

3. **Adjust your package.xml as needed**


CANopen configuration creation
------------------------------

1. **Gather required information**
    Gather all EDS files of the devices connected to the bus and store them
    in your configuration package. 

2. **Writing your bus.yml file** 
    First create the configuration yml file.
    .. code-block:: console

      $ touch bus.yml

    Open the file in the editor of your choice and create the master description.
    .. code-block:: 

      master:
        node_id: [node id]
    
    And add other configuration data as necessary. A documentation of configuration options
    available can be found here `here`_.

    Once you have defined the configuration of your master, add your slaves. The following
    describes the mandatory data per slave. Further configuration options can be found `here`_.

    .. code-block:: 

      [unique slave name]:
        node_id: [node id]
        package: [ros2 package where to find the driver] 
        driver: [qualified name of the driver]

.. _here: https://opensource.lely.com/canopen/docs/dcf-tools/
 

3. **Generating your master.dcf file**
    .. code-block:: console

      $ dcfgen -r -S bus.yml

    This will create your master.dcf as well as a number of .bin files depending on
    your bus.yml. The .bin files are concise dcf files that are used by the can master
    to configure itself and the devices on the bus.




Launch configuration creation
-----------------------------

Create a launch folder in your package directory and a launch file.

.. code-block:: console

  mkdir launch
  touch bring_up.launch.py

**Option1: Lazy load device drivers using Components**:

The device manager extends the component_manager and can be used as a ComposableNodeContainer in your Launchfile.
In your Launchfile you can the choose, which of the slaves you defined in your bus description you want to actually
run.

.. code-block:: python

  def generate_launch_description():
      """Generate launch description with multiple components."""
      container = ComposableNodeContainer(
              name='[container node name]',
              namespace='',
              package='canopen_core',
              executable='device_manager',
              parameters= [{
                          "yaml_path": os.path.join(path_to_test, ".." ,  "resources" , "bus.yml"),
                          "dcf_path": os.path.join(path_to_test, ".." , "resources" , "master.dcf"),
                      }
              ],
              composable_node_descriptions=[
                  ComposableNode(
                      package='proxy_driver',
                      plugin='ros2_canopen::ProxyDriver',
                      name='talker'),
                  ComposableNode(
                      package='composition',
                      plugin='composition::Listener',
                      name='listener')
              ],
              output='screen',
      )

**Option2: Load all drivers on start-up**:

By setting parameter lazy_load_enabled to false, all drivers will be loaded on start-up.
The Device Manager can be started as normal node instead of ComposableNodeContainer.