/*
 * Copyright 2019 Fraunhofer
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SOCKETCAN_INTERFACE_READER_H
#define SOCKETCAN_INTERFACE_READER_H

#include <socketcan_interface/interface.h>
#include <deque>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono.hpp>

namespace can
{

class BufferedReader
{
  std::deque<can::Frame> buffer_;
  boost::mutex mutex_;
  boost::condition_variable cond_;
  CommInterface::FrameListenerConstSharedPtr listener_;
  bool enabled_;
  size_t max_len_;

  void trim()
  {
    if (max_len_ > 0)
    {
      while (buffer_.size() > max_len_)
      {
        ROSCANOPEN_ERROR("socketcan_interface", "buffer overflow, discarded oldest message " /*<< tostring(buffer_.front())*/); // enable message printing
        buffer_.pop_front();
      }
    }
  }
  void handleFrame(const can::Frame & msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (enabled_)
    {
      buffer_.push_back(msg);
      trim();
      cond_.notify_one();
    }
    else
    {
      ROSCANOPEN_WARN("socketcan_interface", "discarded message " /*<< tostring(msg)*/); // enable message printing
    }
  }
  void listen(CommInterfaceSharedPtr interface){
      boost::mutex::scoped_lock lock(mutex_);
      listener_ = interface->createMsgListenerM(this, &BufferedReader::handleFrame);
      buffer_.clear();
  }
  void listen(CommInterfaceSharedPtr interface, const Frame::Header& h){
      boost::mutex::scoped_lock lock(mutex_);
      listener_ = interface->createMsgListenerM(h, this, &BufferedReader::handleFrame);
      buffer_.clear();
public:
  class ScopedEnabler
  {
    BufferedReader &reader_;
    bool before_;
  public:
    ScopedEnabler(BufferedReader &reader) : reader_(reader), before_(reader_.setEnabled(true)) {}
    ~ScopedEnabler()
    {
      reader_.setEnabled(before_);
    }
  };

  BufferedReader() : enabled_(true), max_len_(0) {}
  BufferedReader(bool enable, size_t max_len = 0) : enabled_(enable), max_len_(max_len) {}

  void flush()
  {
    boost::mutex::scoped_lock lock(mutex_);
    buffer_.clear();
  }
  void setMaxLen(size_t max_len)
  {
    boost::mutex::scoped_lock lock(mutex_);
    max_len_ = max_len;
    trim();
  }
  bool isEnabled()
  {
    boost::mutex::scoped_lock lock(mutex_);
    return enabled_;
  }
  bool setEnabled(bool enabled)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bool  before = enabled_;
    enabled_ = enabled;
    return before;
  }
  void enable()
  {
    boost::mutex::scoped_lock lock(mutex_);
    enabled_ = true;
  }

  void disable()
  {
    boost::mutex::scoped_lock lock(mutex_);
    enabled_ = false;
  }

  void listen(CommInterfaceSharedPtr interface)
  {
    boost::mutex::scoped_lock lock(mutex_);
    listener_ = interface->createMsgListener(CommInterface::FrameDelegate(this, &BufferedReader::handleFrame));
    buffer_.clear();
  }
  void listen(CommInterfaceSharedPtr interface, const Frame::Header& h)
  {
    boost::mutex::scoped_lock lock(mutex_);
    listener_ = interface->createMsgListener(h, CommInterface::FrameDelegate(this, &BufferedReader::handleFrame));
    buffer_.clear();
  }

  template<typename DurationType> bool read(can::Frame * msg, const DurationType &duration)
  {
    return readUntil(msg, boost::chrono::high_resolution_clock::now() + duration);
  }
  bool readUntil(can::Frame * msg, boost::chrono::high_resolution_clock::time_point abs_time)
  {
    boost::mutex::scoped_lock lock(mutex_);

    while (buffer_.empty() && cond_.wait_until(lock, abs_time)  != boost::cv_status::timeout)
    {}

    if (buffer_.empty())
    {
      return false;
    }

    if (msg)
    {
      *msg = buffer_.front();
      buffer_.pop_front();
    }
    return true;
  }

};

} // namespace can

#endif  // SOCKETCAN_INTERFACE_READER_H
