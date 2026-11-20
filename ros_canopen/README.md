## Overview

The packages provides support for CANopen devices within ROS.

It can be divided into different parts:

- CAN layer abstraction
- CANopen master with device/object management
- Profile-specific support, currently only for 402 profile (drives and
  motion control)
- ROS interfaces, motion control through ros_control

![ros canopen package structure](doc/ros_canopen_structure.png)

## Tutorials

- [Using maxon EPOS4 in Profile Operating Modes](#using-maxon-epos4-in-profile-operating-modes):
  This tutorial explains how to use Profile Modes of maxon EPOS4
  controller such as Profile Position Mode and Profile Velocity Mode
  with ros_canopen for a single motor.
- [Using maxon EPOS4 in Cyclic Synchronous Operating Modes](#using-maxon-epos4-in-cyclic-synchronous-operating-modes):
  This tutorial explains how to use Cyclic Synchronous Modes of maxon
  EPOS4 controller such as Cyclic Synchronous Position Mode with
  ros_canopen and MoveIt for a pair of motors.

## Using maxon EPOS4 in Profile Operating Modes

- **Note:** This tutorial was tested with ROS Melodic and assumes that
  you have some basic knowledge of ROS and CANopen. Take care of the
  wiring and safety instructions mentioned by the "Hardware Reference"
  of your EPOS4 controller!.
- **Description:** This tutorial explains how to use Profile Modes of
  maxon EPOS4 controller such as Profile Position Mode and Profile
  Velocity Mode with ros_canopen for a single motor.
- **Keywords:** canopen, ros_canopen, epos4, maxon, profile position,
  profile velocity
- **Tutorial Level:** INTERMEDIATE
- **Next Tutorial:**
  [Using maxon EPOS4 in Cyclic Synchronous Operating Modes](#using-maxon-epos4-in-cyclic-synchronous-operating-modes)



```wiki
Please adapt the code to the needs of your concrete application.
Any warranty for proper functionality is excluded and has to be ensured based on tests within the concrete system environment.
Take care of the wiring and safety instructions mentioned by the "Hardware Reference" of the corresponding controller!
```



This tutorial explains how to use maxon EPOS4 controller with
ros_canopen. It should work with any combination of EPOS4 version and
motor, as long as the motor dependent parameters are set properly. For
simplification, only a single EPOS4 controller will be used. However the
following configuration files can be easily extended to multi-axis
robotic systems.

For more information and specific use cases using NVIDIA Jetson TX2 and
Raspberry Pi, along with troubleshooting tips please visit this
[github repository](https://github.com/Roboprotos/maxon_epos4_ros1).

### EPOS4 setup

#### Hardware setup

For wiring, please refer to the official maxon EPOS4 Hardware Reference
manual corresponding to your controller. You will find it in the
Download section after selecting your
[controller](http://epos.maxongroup.com). It is recommended to keep the
automatic bit rate detection on the DIP switch SW1 (factory default).
Don't forget to set the bus termination (7th switch to ON) on your last
node.

#### Software setup with EPOS Studio

To get EPOS Studio software use the "Download EPOS Setup" button at the
bottom of [maxon EPOS website](http://epos.maxongroup.com). It is
recommended to update your EPOS4 with the latest firmware. After setting
up your EPOS4 with the Startup Wizard please proceed to the regulation
tuning.

Finally the CANopen Wizard will help you setting up the PDO mapping that
is expected by ros_canopen. On the Receive PDOs window, verify the
COB-IDs, check "valid" and set Transmission to "synchronous" for all
4 PDOs. Finally set the following mapping:

RxPDO 1

- Controlword
- Target velocity RxPDO 2
- Target position
- Profile velocity RxPDO 3
- Profile acceleration
- Profile deceleration RxPDO 4
- Target torque
- Modes of operation

On the Transmit PDOs window, verify the COB-IDs, check both "valid"
and "RTR" and set Transmission to "synchronous" for all 4 PDOs.
Finally you can set the following mapping:

TxPDO 1

- Statusword
- Modes of operation display TxPDO 2
- Velocity demand value
- Velocity actual value TxPDO 3
- Position actual value
- Velocity actual value averaged TxPDO 4 (empty)

In the Error Protocol tab, you can leave the Heartbeat consumer and
producer unchecked.

You will need to export a DCF file. For that just right click on your
EPOS4 controller, go to Wizards -> Parameter Import/Export, type a file
name and do "Export Parameters to File".

The next steps assume that there is a SocketCAN interface present on the
linux system where ROS is installed. For more information you can follow
the steps explained
[here](http://wiki.ros.org/socketcan_interface#Preparation).

### ROS package setup

You can get a package example from this
[Github repository](https://github.com/Roboprotos/maxon_epos4_ros1/tree/master/maxon_epos4_ros_canopen)
or create your own using the following steps.

#### Package structure

You can create a new ROS package in your workspace with the following
dependencies:



```
$ catkin_create_pkg epos4_canopen std_msgs std_srvs rospy roscpp roslib
```



Create the following directories in your new package:

- `config`
- `launch`
- `urdf`

#### Configuration files

For ros_canopen to function properly, you need to define some parameters
in YAML files.

##### CANopen Bus parameters

In the `config` folder you can create a configuration file called
`can.yaml` and copy/paste the following content:



```text_yaml
bus:
  device: can0 # SocketCAN network interface
#  loopback: false # Make the SocketCAN loop back messages
#  driver_plugin: can::SocketCANInterface
  master_allocator: canopen::SimpleMaster::Allocator
sync:
  interval_ms: 10 # 10 ms is recommended for non real-time systems, set to 0 to disable sync
#  update_ms: <interval_ms> # Update interval of control loop, must be set explicitly if sync is disabled
  overflow: 0 # overflow sync counter at value or do not set it (0, default)
heartbeat: # simple heartbeat producer, optional!
  rate: 20 # heartbeat rate
  msg: "77f#05" # message to send, cansend format: heartbeat of node 127 with status 5=Started
```



This file allows you to specify which Linux CAN interface to use, for
example can0. It also allows you to set a SYNC message (more details of
its usage will be seen in the next tutorial covering Cyclic Synchronous
Modes) and Heartbeat.

##### Controller parameters

Depending on if you are using Profile Position Mode or Profile Velocity
Mode, chose the correct file to use. You can of course mix the two types
of controllers in the same YAML file later on.

Create a configuration file called `controller.yaml` and copy/paste
the chosen content among the two types of Profile Modes:

- Profile Position Mode 

```text_yaml
joint_names: [base_link1_joint]

joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50

joint_group_position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
    - base_link1_joint
  required_drive_mode: 1

base_link1_joint_position_controller:
  type: position_controllers/JointPositionController
  joint: base_link1_joint
  required_drive_mode: 1
```



For Profile Position Mode set the `required_drive_mode` to 1 as
explained in
[canopen_402 wiki page](http://wiki.ros.org/canopen_402#Drive_operation_modes).

- Profile Velocity Mode 

```text_yaml
joint_names: [base_link1_joint]

joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50

joint_group_velocity_controller:
  type: velocity_controllers/JointGroupVelocityController
  joints:
     - base_link1_joint
  required_drive_mode: 3

base_link1_joint_velocity_controller:
  type: velocity_controllers/JointVelocityController
  joint: base_link1_joint
  required_drive_mode: 3
```



For Profile Velocity Mode set the `required_drive_mode` to 3 as
explained in
[canopen_402 wiki page](http://wiki.ros.org/canopen_402#Drive_operation_modes).

Be careful that the joint name should correspond to the one defined in
the URDF and launch file explained later in this tutorial.

##### Node parameters

Create a configuration file called `node.yaml` and copy/paste the
following content:



```text_yaml
nodes:
  node1:
    id: 1
    name: base_link1_joint
    eds_pkg: epos4_ros_canopen # optionals package  name for relative path
    eds_file: "config/epos4_node1.dcf" # path to EDS/DCF file

defaults: # optional, all defaults can be overwritten per node
#  eds_pkg: my_config_package # optional package  name for relative path
#  eds_file: "my_config.dcf" # path to EDS/DCF file
#   dcf_overlay: # "ObjectID": "ParameterValue" (both as strings)
#    "6098": "0" # No homing operation required
#    "1016sub1" : "0x7F0064" # heartbeat timeout of 100 ms for master at 127
#    "1017": "100" # heartbeat producer

# canopen_chain_node settings ..
  motor_allocator: canopen::Motor402::Allocator # select allocator for motor layer
#   motor_layer: settings passed to motor layer (plugin-specific)
  switching_state: 2 # (Operation_Enable), state for mode switching. Drive mode of operation from canopen_402 wiki
  pos_to_device: "pos" # inc
  pos_from_device: "obj6064" # inc 
  vel_to_device: "vel" # rpm
  vel_from_device: "obj606C" # rpm
  eff_to_device: "rint(eff)" # just round to integer
  eff_from_device: "0" # unset
```



Make sure to replace `eds_pkg` and `eds_file` with your package and
file name.

In case you are using Homing Mode, you can use the `dcf_overlay` to
set some parameters. For example, if you are using the `Homing method`
-3 (Current Threshold Positive Speed), then you could set the following
parameters (please adapt the values to your motor):



```text_yaml
defaults: # optional, all defaults can be overwritten per node
  dcf_overlay: # "ObjectID": "ParameterValue" (both as strings)
# Homing mode settings
    "6065": "2000" # Following error window 
    "30B1": "1000" # Home offset move distance 
    "607F": "1300" # Max profile velocity 
    "6085": "10000" # Quick stop deceleration 
    "6099sub1": "100" # Speed for switch search 
    "6099sub2": "10" # Speed for zero search 
    "609A": "1000" # Homing acceleration 
    "30B2": "1500" # Current threshold for homing mode
```



Finally, for simplicity they are no unit conversions applied, so that it
will be possible to send and read "inc" positions and "rpm"
velocities. We will see how to write unit conversions taking care of the
gearbox ratio and encoder resolution in the next tutorial.

##### ROS parameters

Optionally you can create a file called `ros.yaml` and copy/paste the
following content: 

```text_yaml
defaults: # optional, all defaults can be overwritten per node
  publish: ["1A00"] # list of objects to be published (one topic per node)
nodes:
  node1:
    # ...
    #publish: ["1003sub0!"] # list of objects to be published (one topic per node)
  node2:
    # ...
```



##### DCF file

Finally, place your DFC file exported previously in that `config`
folder.

The current official version of `ros_canopen` can't parse boolean
!DataType (0x1) objects. It is therefore necessary to delete the only
boolean !DataType object, \[2200sub2\] (Internal valid logic supply),
then change the !SubNumber of \[2200\] to 2 and !ParameterValue of
\[2200sub0\] to 1:

Your original DCF section for object \[2200\] must look like this:


```
[2200]
SubNumber=3
ParameterName=Power supply
ObjectType=0x9

[2200sub0]
ParameterName=Highest sub-index supported
ObjectType=0x7
DataType=0x5
AccessType=ro
DefaultValue=2
PDOMapping=0
ObjFlags=1
ParameterValue=2

[2200sub1]
ParameterName=Power supply voltage
ObjectType=0x7
DataType=0x6
AccessType=ro
PDOMapping=0
ObjFlags=3

[2200sub2]
ParameterName=Internal valid logic supply
ObjectType=0x7
DataType=0x1
AccessType=ro
PDOMapping=0
ObjFlags=1
```



And you need to change it to this: 

```
[2200]
SubNumber=2
ParameterName=Power supply
ObjectType=0x9

[2200sub0]
ParameterName=Highest sub-index supported
ObjectType=0x7
DataType=0x5
AccessType=ro
DefaultValue=2
PDOMapping=0
ObjFlags=1
ParameterValue=1

[2200sub1]
ParameterName=Power supply voltage
ObjectType=0x7
DataType=0x6
AccessType=ro
PDOMapping=0
ObjFlags=3
```



The last thing to modify is the Homing method `DefaultValue` in case
you are not using a Homing Mode. In that case, change the default and
parameter values from 7 to 0:



```
[6098]
ParameterName=Homing method
ObjectType=0x7
DataType=0x2
AccessType=rww
DefaultValue=0
PDOMapping=0
ObjFlags=0
ParameterValue=0
```



The Homing method value 0 means "no Homing method required" according to
CiA 402 standard.

#### URDF/XACRO file

In the `urdf` folder you can create a configuration file called
`maxon_epos4.xacro` and copy/paste the following content:



```xml
<?xml version="1.0"?>
<!-- maxon EPOS4 example with 1 DOF -->
<robot name="maxon_epos4" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base Link -->
  <link name="base_link">
  </link>

  <!-- Link1 Link -->
  <link name="link1_link">
  </link>

  <!-- Joint between Base Link and Link1 Link -->
  <joint name="base_link1_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <!--dynamics damping="0.7"/-->
    <limit effort="100.0" velocity="100000" lower="-100000.0" upper="100000.0"/>
  </joint>

  <transmission name="transmission_base_link1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="base_link1_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="link1_maxon_motor">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```



For a joint which is controlled in position you can set it to
`revolute` type. If you have a joint controlled in velocity such as a
wheel, you can set it to `continuous` type and remove the lower and
upper limit:



```xml
<!-- Joint between Base link and Link1 link -->
<joint name="base_link1_joint" type="continuous">
  <parent link="base_link"/>
  <child link="link1_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <dynamics damping="0.7"/>
  <limit effort="100.0" velocity="1300"/>
</joint>
```



The joint must be defined according to the one used in controller YAML
file. It is possible to set a velocity limit and upper/lower position
limits, which have to match with the units used in ROS and defined in
the node YAML file. More information can be found
[here](http://wiki.ros.org/canopen_motor_node#unit_conversions).

#### Launch file

In the `launch` folder you can create a launch file called
`maxon_epos4.launch` and copy/paste the following content:



```xml
<?xml version="1.0"?>
<launch>
  <group ns="/maxon">
    <!-- Load the URDF into ROS parameter server -->
    <param name="/maxon/robot_description" command="$(find xacro)/xacro '$(find epos4_canopen)/urdf/maxon_epos4.xacro'"/>

    <node name="canopen_motor" pkg="canopen_motor_node" type="canopen_motor_node" output="screen" clear_params="true" >
      <rosparam command="load" file="$(find epos4_canopen)/config/can.yaml" />
      <rosparam command="load" file="$(find epos4_canopen)/config/controller.yaml" />
      <rosparam command="load" file="$(find epos4_canopen)/config/node.yaml" />
    </node>

    <!-- load the controllers -->
    <node name="controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false"
      output="screen" args="spawn 
      /maxon/canopen_motor/joint_state_controller
      /maxon/canopen_motor/base_link1_joint_position_controller
    "/>
  </group>
</launch>
```



In case you are using a ROS Velocity Controller, you need to change the
last argument of the `controller_spawner` node according to your
`controller.yaml` file:



```xml
<node name="controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false"
  output="screen" args="spawn 
  /maxon/canopen_motor/joint_state_controller
  /maxon/canopen_motor/base_link1_joint_velocity_controller
"/>
```



### Basic demo

Once your ROS package is ready it is time to test it.

#### Installation

To install `ros_canopen` with its dependencies along with useful
controller packages, install the following ROS packages (replace
`melodic` with your ROS version name if necessary):



```
$ sudo apt-get install ros-melodic-canopen-* ros-melodic-control* ros-melodic-rqt-controller-manager ros-melodic-joint-state-controller ros-melodic-velocity-controllers ros-melodic-effort-controllers ros-melodic-joint-state-publisher-gui ros-melodic-std-srvs
```



Make sure your CAN interface is ready, and if not you can for example
bring it up this way: 

```
$ sudo /sbin/ip link set can0 up type can bitrate 1000000
```



#### Sending motor commands

Open a terminal and launch your launch file: 

```
$ roslaunch epos4_canopen maxon_epos4.launch
```



In another terminal, call the initialization service of your driver:


```
$ rosservice call /maxon/driver/init
```

 It should output "`success: True`" if it
worked properly.

You can then send a command to the motor in a third terminal. This is
done by publishing data on a controller topic.

If you have set a position controller, execute the following command,
which will send a Target Position of 10000 inc (if "inc" is the chosen
position unit on the ROS side as explained earlier) to the first node:


```
$ rostopic pub /maxon/canopen_motor/base_link1_joint_position_controller/command std_msgs/Float64 -- 10000
```



If you have set a velocity controller, execute the following command,
which will send a Target Velocity of 500 rpm (if "rpm" is the chosen
velocity unit on the ROS side as explained earlier) to the first node:


```
$ rostopic pub /maxon/canopen_motor/base_link1_joint_velocity_controller/command std_msgs/Float64 -- 500
```



#### Reading motor data

You can display in a terminal the content of the joint states using the
echo command from rostopic: 

```
$ rostopic echo /maxon/joint_states
```

 It is published at
the frequency chosen in the `publishing_rate` parameter of the
`joint_state_controller` in the `controller.yaml` file.

#### Other services

You can halt, recover or shutdown the driver using the following
services:



```
$ rosservice call /maxon/driver/halt
$ rosservice call /maxon/driver/recover
$ rosservice call /maxon/driver/shutdown
```



You can send SDO requests for setting and getting objects. For example,
the following command sets the Profile Velocity (object 0x6081) of the
joint "base_link1_joint" to the value of 1000 rpm:


```
$ rosservice call /maxon/driver/set_object base_link1_joint '!!str 6081' '!!str 1000' false
```



The following command gets the Profile Velocity value of the same node:


```
$ rosservice call /maxon/driver/get_object base_link1_joint '!!str 6081' false
```



To get (similarly set with set_object) the value of an object subindex
you can do like that (example with Speed for Zero Search, object 0x6099
and subindex 0x02):



```
$ rosservice call /maxon/driver/get_object base_link1_joint 6099sub02 false
```



You can get the current state of each controller by calling the
following service: 

```
rosservice call /maxon/controller_manager/list_controllers
```



In this tutorial we saw how to connect maxon EPOS4 with ros_canopen,
send motor commands for Profile Modes and read motor data. In the next
tutorial we are going to use
[Cyclic Synchronous Operating Modes](#using-maxon-epos4-in-cyclic-synchronous-operating-modes).

## Using maxon EPOS4 in Cyclic Synchronous Operating Modes

- **Note:** This tutorial was tested with ROS Melodic and assumes that
  you have some basic knowledge of ROS and CANopen. It is recommended to
  follow the first tutorial on
  [Profile Operating Modes](#using-maxon-epos4-in-profile-operating-modes)
  before going through this tutorial. Take care of the wiring and safety
  instructions mentioned by the "Hardware Reference" of your EPOS4
  controller!.
- **Description:** This tutorial explains how to use Cyclic Synchronous
  Modes of maxon EPOS4 controller such as Cyclic Synchronous Position
  Mode with ros_canopen and MoveIt for a pair of motors.
- **Keywords:** canopen, ros_canopen, epos4, maxon, cyclic synchronous
  position, CSP
- **Tutorial Level:** ADVANCED



```wiki
Please adapt the code to the needs of your concrete application.
Any warranty for proper functionality is excluded and has to be ensured based on tests within the concrete system environment.
Take care of the wiring and safety instructions mentioned by the "Hardware Reference" of the corresponding controller!
```



When you have a robot with a kinematic chain it might be better to use a
motion planner and set the controllers in Cyclic Synchronous Operating
Mode to guaranty that all joints are kept synchronized with the CAN
master.

This tutorial explains how to use maxon EPOS4 controller with
ros_canopen and MoveIt in Cyclic Synchronous Position (CSP). It should
work with any combination of EPOS4 version and motor, as long as the
motor dependent parameters are set properly. For simplification, only a
pair of EPOS4 controllers will be used. However the following
configuration files can be easily extended to more complex robotic
systems.

For more information and specific use cases using NVIDIA Jetson TX2 and
Raspberry Pi, along with troubleshooting tips please visit this
[github repository](https://github.com/Roboprotos/maxon_epos4_ros1).

### EPOS4 setup

Please follow the information from the previous tutorial about
[Profile Operating Modes](#using-maxon-epos4-in-profile-operating-modes)

### ROS packages setup

We will start from the package you created during the previous tutorial
(Profile Operating Modes) and make the necessary modifications to use
CSP.

#### ros_canopen config package

##### CANopen Bus parameters

The `can.yaml` file can be kept the same. There is nonetheless a very
important parameter that you might need to tune, which is the `sync:
interval_ms` and which has to correspond to the "Interpolation time
period value" as seen later in the `node.yaml` file. It is
recommended to set it at 10ms for non real-time system.

##### Controller parameters

For the `controller.yaml` file, you need to add a second joint, define
a `joint_trajectory_controller` and set the `required_drive_mode` to
8 for CSP as explained in
[canopen_402 wiki page](http://wiki.ros.org/canopen_402#Drive_operation_modes):



```text_yaml
joint_names: [base_link1_joint, link1_link2_joint]

joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50

joint_trajectory_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - base_link1_joint
    - link1_link2_joint
  required_drive_mode: 8
  constraints:
    stopped_velocity_tolerance: 20
    base_link1_joint: {trajectory: 50, goal: 50}
    link1_link2_joint: {trajectory: 50, goal: 50}
```



The
[joint trajectory controller](http://wiki.ros.org/joint_trajectory_controller)
holds together all the joints that need to move as a group, following a
motion planner trajectory, like the ones used with MoveIt.

##### Node parameters

For the `node.yaml` file you need to add the second node, set some
parameters for CSP and make sure the unit conversions are correct so
that MoveIt can send proper commands to your EPOS4 controller and read
correct motor data:



```text_yaml
nodes:
  node1:
    id: 1
    name: base_link1_joint
    eds_pkg: maxon_epos4_ros_canopen # optionals package  name for relative path
    eds_file: "config/epos4_50_15_can_ec90flat_gp52c_mile800_node1.dcf" # path to EDS/DCF file
  node2:
    id: 2
    name: link1_link2_joint
    eds_pkg: maxon_epos4_ros_canopen # optionals package  name for relative path
    eds_file: "config/epos4_50_15_can_ec90flat_gp52c_mile800_node2.dcf" # path to EDS/DCF file

defaults: # optional, all defaults can be overwritten per node
#  eds_pkg: my_config_package # optional package  name for relative path
#  eds_file: "my_config.dcf" # path to EDS/DCF file
  dcf_overlay: # "ObjectID": "ParameterValue" (both as strings)
    "60C2sub1": "10" # Interpolation time period value, to be set as the sync/interval_ms value present in canopen_bus_layer.yaml file, 10ms is recommended 

# canopen_chain_node settings ..
  motor_allocator: canopen::Motor402::Allocator # select allocator for motor layer
#   motor_layer: settings passed to motor layer (plugin-specific)
  switching_state: 2 # (Operation_Enable), state for mode switching. Drive mode of operation from canopen_402 wiki
  pos_to_device: "rint(rad2deg(pos)*3200*26/360)" # rad -> inc, for a MILE 800 CPT encoder and a 26:1 gearbox, 3200 = 4*800
  pos_from_device: "deg2rad(obj6064*360/3200/26)" # actual position [inc] -> rad
  vel_to_device: "vel" # rad/s -> mdeg/s
  vel_from_device: "obj606C" # actual velocity [mdeg/s] -> rad/s
  eff_to_device: "rint(eff)" # just round to integer
  eff_from_device: "0" # unset
```



You can see that the `dcf_overlay` of object "Interpolation time
period value" (60C2sub1) is defined to 10, which correspond to the
`sync: interval_ms` defined in `can.yaml` file.



```wiki
This is very important to achieve a smooth motion in between consecutive “Target Position” data sent by the motion library such as MoveIt.
```



Finally as mentioned above, you need to take a special care to the unit
conversions. Let's take a closer look at the following calculation :



```
pos_to_device: "rint(rad2deg(pos)*3200*26/360)" # rad -> inc, for a MILE 800 CPT encoder and a 26:1 gearbox, 3200 = 4*800
```



The MoveIt planner is going to send a joint angle position command
expressed in radian. The EPOS4 is expecting an incremental encoder
position. First, the `rad2deg(pos)` function converts the position in
radian to a position in degree. It is then multiplied by 3200 which
corresponds to the encoder cpt (counts per turn) times the number of
quadcounts (distinctive states per pulse) of the MILE 800 CPT encoder
used in this example. We need to take into account the gearbox reduction
of 26:1, hence the multiplication by 26, corresponding to the GP52 C
used in the tests. We also need to relate that to a whole turn which is
360 degrees. Finally the `rint` function rounds the calculation to the
nearest integer. The `pos_from_device` is calculated in a similar way,
changing an incremental encoder position to an angle in radian.



```wiki
You need to look at the datasheet of your encoder and gearbox and change the unit calculation accordingly!
```



##### URDF file

You'll need to edit the previous URDF file to add a link and joint. The
joint limits are defined in radian and the maximum velocity in rpm:



```text_yaml
<?xml version="1.0"?>
<!-- maxon EPOS4 example with 2 DOF -->
<robot name="maxon_epos4" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
  </link>

  <!-- Link1 link -->
  <link name="link1_link">
  </link>

  <!-- Joint between Base link and Link1 link -->
  <joint name="base_link1_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link1_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.7"/>
    <limit effort="100.0" velocity="1300" lower="${-pi}" upper="${pi}"/>
  </joint>

  <!-- Link2 Link -->
  <link name="link2_link">
  </link>

  <!-- Joint between Link1 link and Link2 link -->
  <joint name="link1_link2_joint" type="revolute">
    <parent link="link1_link"/>
    <child link="link2_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <dynamics damping="0.7"/>
    <limit effort="100.0" velocity="1300" lower="${-pi}" upper="${pi}"/>
  </joint>

  <transmission name="transmission_base_link1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="base_link1_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="link1_maxon_motor">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>26</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="transmission_link1_link2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="link1_link2_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="link2_maxon_motor">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
      <mechanicalReduction>26</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```



##### Launch file

For the launch file you just need to define correctly the controllers:



```
<!-- Load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="controller_manager" respawn="false"
    output="screen" args="spawn 
    /maxon/canopen_motor/joint_state_controller
    /maxon/canopen_motor/joint_trajectory_controller
  "/>
```



#### MoveIt config package

It is recommended to generate MoveIt configuration package using the
Setup Assistant tool so that it fits your project. However, for the
purpose of this tutorial using 2 EPOS4 controllers, you can clone the
[following package](https://github.com/Roboprotos/maxon_epos4_ros1/tree/master/maxon_epos4_moveit_config).
This package has been simplified to use only one motion planning library
(OMPL) and has the minimum files and settings to interconnect both
ros_canopen and MoveIt nodes.

##### Configuration files

- The `joint_limits.yaml` file can be used to overwrite the joint
  velocity limits defined in the URDF file. Explanation on Time
  Parameterization can be read on the
  [following link](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/time_parameterization/time_parameterization_tutorial.html).

<!-- -->

- The `kinematics.yaml` file defines which kinematic solver is being
  used, as explained
  [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/kinematics_configuration/kinematics_configuration_tutorial.html).

<!-- -->

- The maxon_epos4.srdf file is generated by the MoveIt Setup Assistant
  from the URDF file. You can find moreinformation about SRDF
  [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html).
  You can define your own named states that you will be able to use
  later when setting motion goals. Joint angles are expressed in radian.

<!-- -->

- The ompl_planning.yaml file contains the settings for all the
  algorithms used by the Open Motion Planning Library, as described
  [here](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ompl_interface/ompl_interface_tutorial.html).

<!-- -->

- The ros_controllers.yaml file defines the robot controllers to create
  a link to the ones defined in the maxon_epos4_ros_canopen package.
  More information can be found
  [here](http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/controller_configuration.html).

##### Launch files

- `demo.launch`: This file runs the main MoveIt demo to be used with
  `maxon_epos4.launch` file from `epos4_canopen` package as
  explained later in this tutorial. It launches the
  [robot_state_publisher](http://wiki.ros.org/robot_state_publisher)
  node, calls the `move_group.launch` file and the
  `moveit_rviz.launch` file for the Rviz GUI
  ([ROS Visualizer](http://wiki.ros.org/rviz)).

<!-- -->

- `maxon_epos4_moveit_controller_manager.launch.xml`: This file mainly
  loads the `ros_controllers.yaml` configuration file into the ROS
  parameter server.

<!-- -->

- move_group.launch: This file includes other files such as
  `planning_context.launch`, `planning_pipeline.launch.xml`,
  `trajectory_execution.launch.xml` and runs the move_group node.

<!-- -->

- `moveit.rviz`: This file contains the Rviz GUI settings used when
  the application opens.

<!-- -->

- `moveit_rviz.launch`: This file is included in the `demo.launch`
  and it opens the Rviz GUI using `moveit.rviz` settings.

<!-- -->

- `ompl_planning_pipeline.launch.xml`: This file loads the
  `ompl_planning.yaml` configuration file.

<!-- -->

- `planning_context.launch`: This file loads the `maxon_epos4.srdf`,
  `joint_limits.yaml` and `kinematics.yaml` configuration files. You
  need to adapt the following line to your ros_canopen configuration
  package name: 

```
<!-- Load universal robot description format (URDF) -->
<param if="$(arg load_robot_description)" name="$(arg robot_description)" command="xacro  '$(find epos4_canopen)/urdf/maxon_epos4.xacro'"/>
```



<!-- -->

- `planning_pipeline.launch.xml`: This file includes the
  `ompl_planning_pipeline.launch.xml` file.

<!-- -->

- `ros_controllers.launch`: This file loads the
  `ros_controllers.yaml` configuration file. The controllers
  themselves are already loaded by the `maxon_epos4.launch` file from
  the `epos4_canopen` package, so the corresponding section has been
  commented out.

<!-- -->

- `setup_assistant.launch`: This file can be used to re-generate the
  whole `maxon_epos4_moveit_config` package with different settings.
  Be careful when using it as it may overwrite specific settings that
  connects to `epos4_canopen`.

<!-- -->

- `trajectory_execution.launch.xml`: This file includes
  `maxon_epos4_moveit_controller_manager.launch.xml` file.

### Basic demo

#### Installation

To install MoveIt you can follow the
[official website instructions](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html)
and then install these additional packages: 

```
$ sudo apt-get install ros-melodic-moveit-planners ros-melodic-moveit-plugins ros-melodic-joint-trajectory-*
```



Setup your CAN interface as explained in the previous tutorial.

#### Executing a path

Open a terminal and launch your `ros_canopen` launch file:


```
$ roslaunch epos4_canopen maxon_epos4.launch
```



In another terminal, call the initialization service of your driver:


```
$ rosservice call /maxon/driver/init
```

 It should output "`success: True`" if it
worked properly.

In a third terminal you can launch the MoveIt demo:


```
$ roslaunch epos4_moveit_config demo.launch rviz_tutorial:=true
```

 It should open the Rviz window with the
MotionPlanning tab on the bottom left. The "Planning" tab should be
selected by default. In the "Query" sub-panel, click on the "Goal State"
drop-down list and choose "pose1". This means the planner will calculate
a path going from the current state to the state called "pose1", which
has been defined previously. You can then click on "Plan" and see if it
could plan the path successfully. You can then click on "Execute" to
send the motor commands in CSP Mode.
