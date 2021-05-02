#ifndef SOCKETCAN_INTERFACE_DUMMY_H
#define SOCKETCAN_INTERFACE_DUMMY_H

#include <deque>
#include <unordered_map>

#include "interface.h"
#include "dispatcher.h"
#include "string.h"
#include "logging.h"
#include "threading.h"
#include <boost/algorithm/string.hpp>


namespace can {

class DummyBus {
public:
    using FrameDispatcher = SimpleDispatcher<CommInterface::FrameListener>;
    using FrameDispatcherSharedPtr = std::shared_ptr<FrameDispatcher>;
    using Buses = std::unordered_map<std::string,  FrameDispatcherSharedPtr>;
private:
    static Buses& get_buses() {
        static Buses buses;
        return buses;
    }
    FrameDispatcherSharedPtr bus_;
public:
    const std::string name;
    DummyBus(const std::string& name) : name(name), bus_(get_buses().emplace(name, std::make_shared<FrameDispatcher>()).first->second) {
    }
    ~DummyBus() {
        get_buses().erase(name);
    }
    class Connection {
    public:
        inline Connection(FrameDispatcherSharedPtr bus, FrameListenerConstSharedPtr listener)
        : bus_(bus), listener_(listener)
        {}
    void dispatch(const Frame & msg){
        bus_->dispatch_filtered(msg, listener_);
    }
    private:
        FrameDispatcherSharedPtr bus_;
        FrameListenerConstSharedPtr listener_;
    };
    using ConnectionSharedPtr = std::shared_ptr<Connection>;
    template <typename Instance, typename Callable> static inline ConnectionSharedPtr connect(const std::string & name, Instance inst, Callable callable) {
        FrameDispatcherSharedPtr bus = get_buses().at(name);
        return std::make_shared<Connection>(bus, bus->createListener(std::bind(callable, inst, std::placeholders::_1)));
    }
};

class DummyInterface : public DriverInterface{
    using FrameDispatcher = FilteredDispatcher<unsigned int, CommInterface::FrameListener>;
    using StateDispatcher = SimpleDispatcher<StateInterface::StateListener>;
    using Map = std::unordered_map<std::string, Frame>;
    FrameDispatcher frame_dispatcher_;
    StateDispatcher state_dispatcher_;
    DummyBus::ConnectionSharedPtr bus_;
    State state_;
    Map map_;
    std::deque<can::Frame> in_;
    bool loopback_;
    bool trace_;
    boost::mutex mutex_;
    boost::condition_variable cond_;

    bool add_noconv(const std::string &k, const Frame &v, bool multi){
        boost::mutex::scoped_lock cond_lock(mutex_);
        if(multi || map_.find(k) == map_.end()){
              map_.insert( std::make_pair(boost::to_lower_copy(k), v));
              return true;
        }
        return false;
    }
    void setDriverState(State::DriverState state){
        boost::mutex::scoped_lock lock(mutex_);
        if(state_.driver_state != state){
            state_.driver_state = state;
            state_dispatcher_.dispatch(state_);
        }
        cond_.notify_all();
    }
    void enqueue(const Frame & msg){
        boost::mutex::scoped_lock cond_lock(mutex_);
        in_.push_back(msg);
        cond_lock.unlock();
        cond_.notify_all();
    }    
public:
    DummyInterface() : loopback_(false), trace_(false) {}
    DummyInterface(bool loopback) : loopback_(loopback), trace_(false) {}
    virtual ~DummyInterface() { shutdown(); }


    bool add(const std::string &k, const Frame &v, bool multi){
        return add_noconv(boost::to_lower_copy(k), v, multi);
    }
    bool add(const Frame &k, const Frame &v, bool multi){
        return add_noconv(tostring(k,true), v, multi);
    }
    bool add(const std::string &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    bool add(const Frame &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    virtual bool send(const Frame & msg){
        if (trace_) {
            ROSCANOPEN_DEBUG("socketcan_interface", "send: " << msg);
        }
        if (loopback_) {
            enqueue(msg);
        }
        try{
            std::pair <Map::iterator, Map::iterator> r = map_.equal_range(tostring(msg, true));
            for (Map::iterator it=r.first; it!=r.second; ++it){
                enqueue(it->second);
            }
        }
        catch(const std::out_of_range &e){
        }
        bus_->dispatch(msg);
        return true;
    }

    virtual FrameListenerConstSharedPtr createMsgListener(const FrameFunc &delegate){
        return frame_dispatcher_.createListener(delegate);
    }
    virtual FrameListenerConstSharedPtr createMsgListener(const Frame::Header&h , const FrameFunc &delegate){
        return frame_dispatcher_.createListener(h.key(), delegate);
    }

    // methods from StateInterface
    virtual bool recover(){return false;};

    virtual State getState(){
        boost::mutex::scoped_lock cond_lock(mutex_);
        return state_;
    }

    virtual void shutdown(){
        flush();
        setDriverState(State::closed);
        bus_.reset();
    };

    virtual bool translateError(unsigned int internal_error, std::string & str){
        if (!internal_error) {
            str = "OK";
            return true;
        }
        return false;
    };

    virtual bool doesLoopBack() const {
        return loopback_;
    };

    void flush(){
        while (true) {
            {
                boost::mutex::scoped_lock cond_lock(mutex_);
                if (in_.empty()) {
                    return;
                }            
            }
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        }
    }

    virtual void run(){
        boost::mutex::scoped_lock cond_lock(mutex_);
        while (true) {

            state_.driver_state = State::ready;
            state_dispatcher_.dispatch(state_);

            cond_.wait(cond_lock);
            while(!in_.empty()){
                const can::Frame msg = in_.front();
                in_.pop_front();
                if (trace_) {
                    ROSCANOPEN_DEBUG("socketcan_interface", "receive: " << msg);
                }
                frame_dispatcher_.dispatch(msg.key(), msg);
            }
            if (state_.driver_state == State::closed) {
                return;
            }
        }
    }

    bool init(const std::string &device, bool loopback){
        loopback_ = loopback;
        bus_ = DummyBus::connect(device, this, &DummyInterface::enqueue);
        setDriverState(State::open);
        return true;
    }

    virtual bool init(const std::string &device, bool loopback, SettingsConstSharedPtr settings) {
        if(DummyInterface::init(device, loopback)) {
            trace_ = settings->get_optional("trace", false);
            return true;
        } else {
            return false;
        }
    }
    virtual StateListenerConstSharedPtr createStateListener(const StateFunc &delegate){
      return state_dispatcher_.createListener(delegate);
    }

};

using DummyInterfaceSharedPtr = std::shared_ptr<DummyInterface>;
using ThreadedDummyInterface = ThreadedInterface<DummyInterface>;
using ThreadedDummyInterfaceSharedPtr = std::shared_ptr<ThreadedDummyInterface>;

}

#endif
