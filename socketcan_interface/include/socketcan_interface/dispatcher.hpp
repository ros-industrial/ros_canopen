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

#ifndef SOCKETCAN_INTERFACE__DISPATCHER_HPP_
#define SOCKETCAN_INTERFACE__DISPATCHER_HPP_

#include <boost/thread/mutex.hpp>

#include <functional>
#include <memory>
#include <list>
#include <unordered_map>

#include "interface.hpp"

namespace can
{

template< typename Listener > class SimpleDispatcher
{
public:
  using Callable = typename Listener::Callable;
  using Type = typename Listener::Type;
  using ListenerConstSharedPtr = typename Listener::ListenerConstSharedPtr;
protected:
  class DispatcherBase;
  using DispatcherBaseSharedPtr = std::shared_ptr<DispatcherBase>;
  class DispatcherBase
  {
    DispatcherBase(const DispatcherBase&) = delete;  // prevent copies
    DispatcherBase& operator=(const DispatcherBase&) = delete;

    class GuardedListener: public Listener
    {
      std::weak_ptr<DispatcherBase> guard_;
    public:
      GuardedListener(DispatcherBaseSharedPtr g, const Callable &callable) :
        Listener(callable), guard_(g) {}
      virtual ~GuardedListener()
      {
        DispatcherBaseSharedPtr d = guard_.lock();
        if (d)
        {
          d->remove(this);
        }
      }
    };

    boost::mutex &mutex_;
    std::list<const Listener* > listeners_;
  public:
    explicit DispatcherBase(boost::mutex &mutex) : mutex_(mutex) {}
    void dispatch_nolock(const Type &obj) const
    {
      for (
        typename std::list<const Listener* >::const_iterator it = listeners_.begin();
        it != listeners_.end();
        ++it)
      {
        (**it)(obj);
      }
    }
    void remove(Listener *d)
    {
      boost::mutex::scoped_lock lock(mutex_);
      listeners_.remove(d);
    }
    size_t numListeners()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return listeners_.size();
    }

    static ListenerConstSharedPtr createListener(
      DispatcherBaseSharedPtr dispatcher, const  Callable &callable)
    {
      ListenerConstSharedPtr l(new GuardedListener(dispatcher, callable));
      dispatcher->listeners_.push_back(l.get());
      return l;
    }
  };
  boost::mutex mutex_;
  DispatcherBaseSharedPtr dispatcher_;
public:
  SimpleDispatcher() : dispatcher_(new DispatcherBase(mutex_)) {}
  ListenerConstSharedPtr createListener(const Callable &callable)
  {
    boost::mutex::scoped_lock lock(mutex_);
    return DispatcherBase::createListener(dispatcher_, callable);
  }
  void dispatch(const Type &obj)
  {
    boost::mutex::scoped_lock lock(mutex_);
    dispatcher_->dispatch_nolock(obj);
  }
  size_t numListeners()
  {
    return dispatcher_->numListeners();
  }
  operator Callable()
  {
    return Callable(this, &SimpleDispatcher::dispatch);
  }
};

template<typename K, typename Listener, typename Hash = std::hash<K>>
class FilteredDispatcher: public SimpleDispatcher<Listener>
{
  using BaseClass = SimpleDispatcher<Listener>;
  std::unordered_map<K, typename BaseClass::DispatcherBaseSharedPtr, Hash> filtered_;
public:
  using BaseClass::createListener;
  template <typename T>
  [[deprecated("provide key explicitly")]]
  typename BaseClass::ListenerConstSharedPtr createListener(const T &key, const typename BaseClass::Callable &callable)
  {
    return createListener(static_cast<K>(key), callable);
  }
  void dispatch(const K &key, const typename BaseClass::Type &obj)
  {
    boost::mutex::scoped_lock lock(BaseClass::mutex_);
    typename BaseClass::DispatcherBaseSharedPtr &ptr = filtered_[key];
    if(ptr) ptr->dispatch_nolock(obj);
    BaseClass::dispatcher_->dispatch_nolock(obj);
  }
  [[deprecated("provide key explicitly")]]
  void dispatch(const typename BaseClass::Type &obj)
  {
    return dispatch(static_cast<K>(obj), obj);
  }
  operator typename BaseClass::Callable()
  {
    return typename BaseClass::Callable(this, &FilteredDispatcher::dispatch);
  }
};

}  // namespace can

#endif  // SOCKETCAN_INTERFACE__DISPATCHER_HPP_
