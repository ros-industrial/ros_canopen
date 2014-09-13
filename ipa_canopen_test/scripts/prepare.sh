#!/bin/bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 500000