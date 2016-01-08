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

#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/string.h>
#include <can_msgs/Frame.h>
#include <string>

namespace socketcan_bridge
{
  SocketCANToTopic::SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      boost::shared_ptr<can::DriverInterface> driver)
    {
      can_topic_ = nh->advertise<can_msgs::Frame>("received_messages", 10);
      driver_ = driver;
    };

  int SocketCANToTopic::setup()
    {
      // register handler for frames and state changes.
      frame_listener_ = driver_->createMsgListener(
              can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback));

      state_listener_ = driver_->createStateListener(
              can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));
    };

  void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
      ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());

      if (f.is_error)
        {
          ROS_WARN("Message is error: %s", can::tostring(f, true).c_str());
        }

      can_msgs::Frame msg;
      msg.id = f.id;
      msg.dlc = f.dlc;
      msg.is_error = f.is_error;
      msg.is_rtr = f.is_rtr;
      msg.is_extended = f.is_extended;

      msg.header.frame_id = "0";  // "0" for no frame.
      msg.header.stamp = ros::Time::now();

      for (int i = 0; i < f.dlc; ++i)
        {
          msg.data[i] = f.data[i];
        }
      can_topic_.publish(msg);
    };


  void SocketCANToTopic::stateCallback(const can::State & s)
    {
      std::string err;
      driver_->translateError(s.internal_error, err);
      ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
    };
};  // namespace socketcan_bridge
