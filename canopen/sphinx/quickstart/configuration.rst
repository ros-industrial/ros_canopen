Creating a Configuration Package
========================================
In order to use the ros2_canopen stack for your robot, you need to
create a configuration package, that holds the configuration of the
bus as well as you launch script. The following sections detail the
steps to create such a package.


Package creation
----------------
When you create your package, you have to first make some decisions.
You will need to choose a package name, decide which bus configurations
you need to have (usually one per CAN interface) and which slaves you have.

| **package_name**: Name of the package
| **bus_config_name**: Name of the the bus configuration (you can have multiple)

You can use `ros2 pkg create` command to create your package.

.. code-block:: console

  $ ros2 pkg create --dependencies ros2_canopen lely_core_libraries --build-type ament_cmake {package_name}
  $ cd {package_name}
  $ rm -rf src
  $ rm -rf include
  $ mkdir -p launch
  $ mkdir -p config

Now your package directory should look like this.

::

      {package_name}
      ├── config
      ├── launch
      ├── CMakeLists.txt
      └── package.xml



Bus configuration creation
------------------------------

#. **Bus configuration decisions** 
    Decide how many bus configurations you need. Add one subfolder for each
    bus configuration in your `config`-folder.

    .. code-block:: bash
      $ mkdir -p {bus_config_name}

#. **Add device information to bus configurations** 
    Once you created the folders for your bus configurations add the .eds files
    for the devices that are in the bus configuration to the respective folder.

    Now your package directory should now look like this.
    ::

          {package_name}
          ├── config
          │   ├── {bus_config_name_1}
          │   |   ├── {device1}.eds
          │   |   ├── {device...}.eds
          │   |   └── {slave_n}.eds
          │   └── {bus_config_name_2}
          │       ├── {device1}.eds
          │       ├── {device...}.eds
          │       └── {slave_n}.eds
          ├── CMakeLists.txt
          └── package.xml

#. **Create the bus configuration specifications** 
    To specify the bus configuration ros2_canopen uses the a YAML-file called
    bus.yml. Create the file in the respective bus configuration folder.
    
    .. code-block:: console

      $ touch bus.yml

    ::

          {package_name}
          ├── config
          │   ├── {bus_config_name_1}
          │   |   ├── bus.yml
          │   |   ├── {device1}.eds
          │   |   ├── {device...}.eds
          │   |   └── {slave_n}.eds
          │   └── {bus_config_name_2}
          │       ├── bus.yml
          │       ├── {device1}.eds
          │       ├── {device...}.eds
          │       └── {slave_n}.eds
          ├── CMakeLists.txt
          └── package.xml

#. **Edit the bus configuration specifications** 
    You need to modify each bus.yml file according to your needs.
    First you need to define where these files and generated files will be
    found at runtime. This is usually the following if you use colcon to
    build from source.

    .. code-block:: yaml
    
      options:
        dcf_path: install/{package_name}/share/{package_name}/config/{bus_config_name}

    Then you need to define your master.
    
    .. code-block:: yaml
    
      master:
        node_id: [node id]
        package: [ros2 package where to find the master driver (usually canopen_core)] 
        driver: [component type of the driver (ros2_canopen::MasterNode or ros2_canopen::LifecycleMasterNode)]
    
    Make sure, that you specify a lifecycle master if you use the lifecycled version of ros2_canopen.
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

    Make sure you use a lifecycle slave if you use the lifecycled version of ros2_canopen.


Launch configuration creation
-----------------------------

Create a launch folder in your package directory and a launch file.

.. code-block:: console

  mkdir launch
  touch {...}.launch.py

Add the following code:

.. code-block:: python

  def generate_launch_description():
        """Generate launch description with multiple components."""
        path_file = os.path.dirname(__file__)

        ld = launch.LaunchDescription()


        device_container = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("canopen_core"), "launch"),
                    "/canopen.launch.py", # if lifecycled operation canope_lifecycle.launch.py
                ]
            ),
            launch_arguments={
                "master_config": os.path.join(
                    get_package_share_directory("{package_name}"),
                    "config",
                    "{bus_config_name}",
                    "master.dcf",
                ),
                "master_bin": os.path.join(
                    get_package_share_directory("{package_name}"),
                    "config",
                    "{bus_config_name}",
                    "master.bin",
                ),
                "bus_config": os.path.join(
                    get_package_share_directory("{package_name}"),
                    "config",
                    "{bus_config_name}",
                    "bus.yml",
                ),
                "can_interface_name": "{can_interface i.e. can0}",
            }.items(),

        )

        ld.add_action(device_container)

        return ld


CMAKE Configuration creation
-----------------------------
Finally we need to adjust the CMakeLists.txt file to pick everything up correctly.

.. code-block:: cmake

  cmake_minimum_required(VERSION 3.8)
  project({package_name})

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(canopen_core REQUIRED)
  find_package(canopen_interfaces REQUIRED)
  find_package(canopen_base_driver REQUIRED)
  find_package(canopen_proxy_driver REQUIRED)
  find_package(lely_core_libraries REQUIRED)


  generate_dcf({bus_config_name})

  install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch/
  )

  install(DIRECTORY
    launch_tests/
    DESTINATION share/${PROJECT_NAME}/launch_tests/
  )


  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  ament_package()






