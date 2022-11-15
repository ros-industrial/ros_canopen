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

#ifndef SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H
#define SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H

#include <socketcan_interface/socketcan.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>

#define _MASK(offset, len) (((1u << ((offset) + (len))) - 1u) ^ ((1u << (offset))-1u))

#define CANMSG_DLC_DLC_OFFSET (0u)
#define CANMSG_DLC_DLC_LEN (7u)
#define CANMSG_DLC_DLC_MASK _MASK(CANMSG_DLC_DLC_OFFSET, CANMSG_DLC_DLC_LEN)

#define CANMSG_DLC_FDFLAGS_OFFSET (CANMSG_DLC_DLC_LEN)
#define CANMSG_DLC_FDFLAGS_LEN (1u)
#define CANMSG_DLC_FDFLAGS_MASK _MASK(CANMSG_DLC_FDFLAGS_OFFSET, CANMSG_DLC_FDFLAGS_LEN)

namespace socketcan_bridge
{
class TopicToSocketCAN
{
  public:
    TopicToSocketCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param, can::DriverInterfaceSharedPtr driver);
    void setup();

  private:
    ros::Subscriber can_topic_;
    can::DriverInterfaceSharedPtr driver_;

    can::StateListenerConstSharedPtr state_listener_;

    void msgCallback(const can_msgs::Frame::ConstPtr& msg);
    void stateCallback(const can::State & s);
};

void convertMessageToSocketCAN(const can_msgs::Frame& m, can::Frame& f)
{
  f.id = m.id;
  f.dlc = (m.dlc & CANMSG_DLC_DLC_MASK) >> CANMSG_DLC_DLC_OFFSET;
  f.is_fd = ((m.dlc & CANMSG_DLC_FDFLAGS_MASK) != 0) || (f.dlc > 8);
  f.flags = (m.dlc & CANMSG_DLC_FDFLAGS_MASK) >> CANMSG_DLC_FDFLAGS_OFFSET;
  f.is_error = m.is_error;
  f.is_rtr = m.is_rtr;
  f.is_extended = m.is_extended;

  int copy_len = m.data.size() > f.data.size() ? f.data.size() : m.data.size();

  for (int i = 0; i < copy_len; i++)
  {
    f.data[i] = m.data[i];
  }
};

};  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H
