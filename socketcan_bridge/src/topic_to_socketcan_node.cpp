/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "topic_to_socketcan_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "can0");

  // get ignored errors from ros parameters
  can::ignored_errors_t ignored_errors;
  nh_param.param<int>("ignore_errors/can_err_tx_timeout", ignored_errors["can_err_tx_timeout"], -1);
  nh_param.param<int>("ignore_errors/can_err_lostarb",    ignored_errors["can_err_lostarb"],    -1);
  nh_param.param<int>("ignore_errors/can_err_ctrl",       ignored_errors["can_err_ctrl"],       -1);
  nh_param.param<int>("ignore_errors/can_err_prot",       ignored_errors["can_err_prot"],       -1);
  nh_param.param<int>("ignore_errors/can_err_trx",        ignored_errors["can_err_trx"],        -1);
  nh_param.param<int>("ignore_errors/can_err_ack",        ignored_errors["can_err_ack"],        -1);
  nh_param.param<int>("ignore_errors/can_err_busoff",     ignored_errors["can_err_busoff"],     -1);
  nh_param.param<int>("ignore_errors/can_err_buserror",   ignored_errors["can_err_buserror"],   -1);
  nh_param.param<int>("ignore_errors/can_err_restarted",  ignored_errors["can_err_restarted"],  -1);

  auto settings = can::SettingsMap();

  settings.set("ignored_errors", ignored_errors);


  can::ThreadedSocketCANInterfaceSharedPtr driver = std::make_shared<can::ThreadedSocketCANInterface> ();

  if (!driver->init(can_device, 0, settings))  // initialize device at can_device, 0 for no loopback.
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device.c_str());
    return 1;
  }
    else
  {
    ROS_INFO("Successfully connected to %s.", can_device.c_str());
  }

  socketcan_bridge::TopicToSocketCAN to_socketcan_bridge(&nh, &nh_param, driver);
  to_socketcan_bridge.setup();

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
