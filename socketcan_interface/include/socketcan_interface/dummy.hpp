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

#ifndef SOCKETCAN_INTERFACE__DUMMY_HPP_
#define SOCKETCAN_INTERFACE__DUMMY_HPP_

#include <boost/algorithm/string.hpp>

#include <unordered_map>
#include <string>
#include <utility>
#include <memory>

#include "interface.hpp"
#include "dispatcher.hpp"
#include "string.hpp"

namespace can
{

class DummyInterface : public DriverInterface
{
  using FrameDispatcher = FilteredDispatcher<unsigned int, CommInterface::FrameListener>;
  using StateDispatcher = SimpleDispatcher<StateInterface::StateListener>;
  using Map = std::unordered_map<std::string, Frame>;
  FrameDispatcher frame_dispatcher_;
  StateDispatcher state_dispatcher_;
  State state_;
  Map map_;
  bool loopback_;

  bool add_noconv(const std::string &k, const Frame &v, bool multi)
  {
    if (multi || map_.find(k) == map_.end())
    {
      map_.insert(std::make_pair(boost::to_lower_copy(k), v));
      return true;
    }
    return false;
  }
public:
  explicit DummyInterface(bool loopback) : loopback_(loopback) {}

  bool add(const std::string &k, const Frame &v, bool multi)
  {
    return add_noconv(boost::to_lower_copy(k), v, multi);
  }
  bool add(const Frame &k, const Frame &v, bool multi)
  {
    return add_noconv(tostring(k, true), v, multi);
  }
  bool add(const std::string &k, const std::string &v, bool multi)
  {
    return add(k, toframe(v), multi);
  }
  bool add(const Frame &k, const std::string &v, bool multi)
  {
    return add(k, toframe(v), multi);
  }
  virtual bool send(const Frame & msg)
  {
    if (loopback_) frame_dispatcher_.dispatch(msg.key(), msg);
    try
    {
      std::pair <Map::iterator, Map::iterator> r = map_.equal_range(tostring(msg, true));
      for (Map::iterator it = r.first; it != r.second; ++it)
      {
        frame_dispatcher_.dispatch(it->second.key(), it->second);
      }
    }
    catch (const std::out_of_range &e)
    {
    }
    return true;
  }

  virtual FrameListenerConstSharedPtr createMsgListener(const FrameFunc &delegate)
  {
      return frame_dispatcher_.createListener(delegate);
  }
  virtual FrameListenerConstSharedPtr createMsgListener(const Frame::Header&h , const FrameFunc &delegate)
  {
      return frame_dispatcher_.createListener(h.key(), delegate);
  }

  // methods from StateInterface
  virtual bool recover()
  {
    return false;
  };

  virtual State getState()
  {
    return state_;
  };

  virtual void shutdown() {}

  virtual bool translateError(unsigned int internal_error, std::string & str)
  {
    if (!internal_error)
    {
      str = "OK";
      return true;
    }
    return false;
  };

  virtual bool doesLoopBack() const
  {
    return loopback_;
  };

  virtual void run() {}

  bool init(const std::string &device, bool loopback)
  {
    loopback_ = loopback;
    state_.driver_state = State::ready;
    state_.internal_error = 0;
    state_dispatcher_.dispatch(state_);
    return true;
  };

  virtual StateListenerConstSharedPtr createStateListener(const StateFunc &delegate)
  {
    return state_dispatcher_.createListener(delegate);
  };
};
using DummyInterfaceSharedPtr = std::shared_ptr<DummyInterface>;

}  // namespace can

#endif  // SOCKETCAN_INTERFACE__DUMMY_HPP_
