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
#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>


#define GET_IGNORE_ERROR_ROS_PARAM(NAME) \
  bool NAME ## _param;\
  if (nh_param.getParam("ignore_errors/"#NAME, NAME ## _param)) {\
    ROS_INFO("Ignoring %s", #NAME); \
    ignored_errors[#NAME] = static_cast<int>(NAME ## _param);\
  } else {\
    ignored_errors[#NAME] = -1;\
  }\


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "socketcan_to_topic_node");

  ros::NodeHandle nh(""), nh_param("~");

  std::string can_device;
  nh_param.param<std::string>("can_device", can_device, "can0");

  // get ignored errors from ros parameters
  can::ignored_errors_t ignored_errors;
  GET_IGNORE_ERROR_ROS_PARAM(can_err_tx_timeout);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_lostarb);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_crtl);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_prot);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_trx);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_ack);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_busoff);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_buserror);
  GET_IGNORE_ERROR_ROS_PARAM(can_err_restarted);

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

  socketcan_bridge::SocketCANToTopic to_topic_bridge(&nh, &nh_param, driver);
  to_topic_bridge.setup(nh_param);

  ros::spin();

  driver->shutdown();
  driver.reset();

  ros::waitForShutdown();
}
