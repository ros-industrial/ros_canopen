#!/bin/bash

trap 'kill $(jobs -p); exit' SIGINT SIGTERM EXIT

echo "prepare"

rosrun controller_manager controller_manager load /rig12/rig1_plate_joint_position_controller __ns:=rig12
rosrun controller_manager controller_manager load /rig12/rig1_plate_joint_velocity_controller __ns:=rig12
rosrun controller_manager controller_manager load /rig12/rig2_plate_joint_position_controller __ns:=rig12
rosrun controller_manager controller_manager load /rig12/rig2_plate_joint_velocity_controller __ns:=rig12

rosrun controller_manager controller_manager stop /rig12/joint_trajectory_controller __ns:=rig12

while true
do
    echo "start"
    rosrun controller_manager controller_manager start /rig12/rig1_plate_joint_position_controller __ns:=rig12
    rosrun controller_manager controller_manager start /rig12/rig2_plate_joint_position_controller __ns:=rig12

    rostopic pub --once /rig12/rig1_plate_joint_position_controller/command std_msgs/Float64 "data: 0" &
    rostopic pub --once /rig12/rig2_plate_joint_position_controller/command std_msgs/Float64 "data: 0" &
    sleep 1
    echo "goto 3.14"
    rostopic pub --once /rig12/rig1_plate_joint_position_controller/command std_msgs/Float64 "data: 3.14" &
    rostopic pub --once /rig12/rig2_plate_joint_position_controller/command std_msgs/Float64 "data: 3.14" &
    sleep 1
    echo "goto 0"
    rostopic pub --once /rig12/rig1_plate_joint_position_controller/command std_msgs/Float64 "data: 0" &
    rostopic pub --once /rig12/rig2_plate_joint_position_controller/command std_msgs/Float64 "data: 0" &
    sleep 1

    rosrun controller_manager controller_manager stop /rig12/rig1_plate_joint_position_controller __ns:=rig12
    rosrun controller_manager controller_manager stop /rig12/rig2_plate_joint_position_controller __ns:=rig12

    rosrun controller_manager controller_manager start /rig12/rig1_plate_joint_velocity_controller __ns:=rig12
    rosrun controller_manager controller_manager start /rig12/rig2_plate_joint_velocity_controller __ns:=rig12

    echo "forward"
    rostopic pub --once /rig12/rig1_plate_joint_velocity_controller/command std_msgs/Float64 "data: 6.28" &
    rostopic pub --once /rig12/rig2_plate_joint_velocity_controller/command std_msgs/Float64 "data: 6.28" &
    sleep 10
    echo "backward"
    rostopic pub --once /rig12/rig1_plate_joint_velocity_controller/command std_msgs/Float64 "data: -6.28" &
    rostopic pub --once /rig12/rig2_plate_joint_velocity_controller/command std_msgs/Float64 "data: -6.28" &
    sleep 10
    echo "stop"
    rostopic pub --once /rig12/rig1_plate_joint_velocity_controller/command std_msgs/Float64 "data: 0" &
    rostopic pub --once /rig12/rig2_plate_joint_velocity_controller/command std_msgs/Float64 "data: 0" &
    sleep 1

    rosrun controller_manager controller_manager stop /rig12/rig1_plate_joint_velocity_controller __ns:=rig12
    rosrun controller_manager controller_manager stop /rig12/rig2_plate_joint_velocity_controller __ns:=rig12
done
  

