#!/bin/bash

device=${1-can0}
shift

bitrate=${1-500000}
shift

sudo modprobe peak_usb
sudo ip link set $device down
sudo ip link set $device up type can bitrate $bitrate $*
