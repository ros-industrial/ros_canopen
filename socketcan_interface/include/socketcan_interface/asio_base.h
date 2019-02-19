// Copyright (c) 2016-2019, Fraunhofer, Mathias Lüdtke, AutonomouStuff
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef SOCKETCAN_INTERFACE__ASIO_BASE_HPP_
#define SOCKETCAN_INTERFACE__ASIO_BASE_HPP_

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <functional>

#include "socketcan_interface/dispatcher.hpp"
#include "socketcan_interface/interface.hpp"

namespace can
{

template<typename Socket>
class AsioDriver : public DriverInterface
{
  using FrameDispatcher = FilteredDispatcher<unsigned int, CommInterface::FrameListener>;
  using StateDispatcher = SimpleDispatcher<StateInterface::StateListener>;
  FrameDispatcher frame_dispatcher_;
  StateDispatcher state_dispatcher_;

  State state_;
  boost::mutex state_mutex_;
  boost::mutex socket_mutex_;

protected:
  boost::asio::io_service io_service_;
#if BOOST_ASIO_VERSION >= 101200  // Boost 1.66+
  boost::asio::io_context::strand strand_;
#else
  boost::asio::strand strand_;
#endif
  Socket socket_;
  Frame input_;

  virtual void triggerReadSome() = 0;
  virtual bool enqueue(const Frame & msg) = 0;

  void dispatchFrame(const Frame & msg)
  {
    strand_.post([this, msg]{ frame_dispatcher_.dispatch(msg.key(), msg);} ); // copies msg
  }
  void setErrorCode(const boost::system::error_code & error)
  {
    boost::mutex::scoped_lock lock(state_mutex_);
    if (state_.error_code != error) {
      state_.error_code = error;
      state_dispatcher_.dispatch(state_);
    }
  }
  void setInternalError(unsigned int internal_error)
  {
    boost::mutex::scoped_lock lock(state_mutex_);
    if (state_.internal_error != internal_error) {
      state_.internal_error = internal_error;
      state_dispatcher_.dispatch(state_);
    }
  }

  void setDriverState(State::DriverState state)
  {
    boost::mutex::scoped_lock lock(state_mutex_);
    if (state_.driver_state != state) {
      state_.driver_state = state;
      state_dispatcher_.dispatch(state_);
    }
  }
  void setNotReady()
  {
    setDriverState(socket_.is_open() ? State::open : State::closed);
  }

  void frameReceived(const boost::system::error_code & error)
  {
    if (!error) {
      dispatchFrame(input_);
      triggerReadSome();
    } else {
      setErrorCode(error);
      setNotReady();
    }
  }

  AsioDriver()
  : strand_(io_service_), socket_(io_service_)
  {}

public:
  virtual ~AsioDriver()
  {
    shutdown();
  }

  State getState()
  {
    boost::mutex::scoped_lock lock(state_mutex_);
    return state_;
  }
  virtual void run()
  {
    setNotReady();

    if (getState().driver_state == State::open) {
      io_service_.reset();
      boost::asio::io_service::work work(io_service_);
      setDriverState(State::ready);

      boost::thread post_thread([this]()
        {
          io_service_.run();
        });

      triggerReadSome();

      boost::system::error_code ec;
      io_service_.run(ec);
      setErrorCode(ec);

      setNotReady();
    }
    state_dispatcher_.dispatch(getState());
  }
  virtual bool send(const Frame & msg)
  {
    return getState().driver_state == State::ready && enqueue(msg);
  }

  virtual void shutdown()
  {
    if (socket_.is_open()) {
      socket_.cancel();
      socket_.close();
    }

    io_service_.stop();
  }

  virtual FrameListenerConstSharedPtr createMsgListener(const FrameFunc & delegate)
  {
    return frame_dispatcher_.createListener(delegate);
  }
  virtual FrameListenerConstSharedPtr createMsgListener(
    const Frame::Header & h, const FrameFunc & delegate)
  {
    return frame_dispatcher_.createListener(h.key(), delegate);
  }
  virtual StateListenerConstSharedPtr createStateListener(const StateFunc & delegate)
  {
    return state_dispatcher_.createListener(delegate);
  }
};

}  // namespace can

#endif  // SOCKETCAN_INTERFACE__ASIO_BASE_HPP_
