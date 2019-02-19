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

#ifndef SOCKETCAN_INTERFACE__THREADING_HPP_
#define SOCKETCAN_INTERFACE__THREADING_HPP_

#include <boost/thread/thread.hpp>

#include "interface.hpp"

namespace can
{

class StateWaiter
{
  boost::mutex mutex_;
  boost::condition_variable cond_;
  can::StateInterface::StateListenerConstSharedPtr state_listener_;
  can::State state_;
  void updateState(const can::State &s)
  {
    boost::mutex::scoped_lock lock(mutex_);
    state_ = s;
    lock.unlock();
    cond_.notify_all();
  }
public:
  template<typename InterfaceType> StateWaiter(InterfaceType *interface)
  {
    state_ = interface->getState();
    state_listener_ = interface->createStateListener(std::bind(&StateWaiter::updateState, this, std::placeholders::_1));
  }
  template<typename DurationType> bool wait(const can::State::DriverState &s, const DurationType &duration)
  {
    boost::mutex::scoped_lock cond_lock(mutex_);
    boost::system_time abs_time = boost::get_system_time() + duration;
    while(s != state_.driver_state)
    {
      if(!cond_.timed_wait(cond_lock,abs_time))
      {
        return false;
      }
    }
    return true;
  }
};

template<typename WrappedInterface> class ThreadedInterface : public WrappedInterface
{
  std::shared_ptr<boost::thread> thread_;
  void run_thread()
  {
    WrappedInterface::run();
  }
public:
  virtual bool init(const std::string &device, bool loopback)
  {
    if (!thread_ && WrappedInterface::init(device, loopback))
    {
      StateWaiter waiter(this);
      thread_.reset(new boost::thread(&ThreadedInterface::run_thread, this));
      return waiter.wait(can::State::ready, boost::posix_time::seconds(1));
    }
    return WrappedInterface::getState().isReady();
  }
  virtual void shutdown()
  {
    WrappedInterface::shutdown();
    if (thread_)
    {
      thread_->interrupt();
      thread_->join();
      thread_.reset();
    }
  }
  void join()
  {
    if (thread_)
    {
      thread_->join();
    }
  }
  virtual ~ThreadedInterface() {}
  ThreadedInterface(): WrappedInterface() {}
  template<typename T1> ThreadedInterface(const T1 &t1): WrappedInterface(t1) {}
  template<typename T1, typename T2> ThreadedInterface(const T1 &t1, const T2 &t2): WrappedInterface(t1, t2) {}

};


}  // namespace can

#endif  // SOCKETCAN_INTERFACE__THREADING_HPP_
