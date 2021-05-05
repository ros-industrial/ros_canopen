#ifndef H_CAN_THREADING_BASE
#define H_CAN_THREADING_BASE

#include <socketcan_interface/interface.h>
#include <boost/thread/thread.hpp>

namespace can{

class StateWaiter{
    boost::mutex mutex_;
    boost::condition_variable cond_;
    can::StateInterface::StateListenerConstSharedPtr state_listener_;
    can::State state_;
    void updateState(const can::State &s){
        boost::mutex::scoped_lock lock(mutex_);
        state_ = s;
        lock.unlock();
        cond_.notify_all();
    }
public:
    template<typename InterfaceType> StateWaiter(InterfaceType *interface){
        state_ = interface->getState();
        state_listener_ = interface->createStateListener(std::bind(&StateWaiter::updateState, this, std::placeholders::_1));
    }
    template<typename DurationType> bool wait(const can::State::DriverState &s, const DurationType &duration){
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

template<typename WrappedInterface> class ThreadedInterface : public WrappedInterface{
    std::shared_ptr<boost::thread> thread_;
    void run_thread(){
        WrappedInterface::run();
    }
    void shutdown_internal(){
        if(thread_){
            thread_->interrupt();
            thread_->join();
            thread_.reset();
        }
    }
public:
    [[deprecated("provide settings explicitly")]] virtual bool init(const std::string &device, bool loopback) override {
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        if(!thread_ && WrappedInterface::init(device, loopback)){
            StateWaiter waiter(this);
            thread_.reset(new boost::thread(&ThreadedInterface::run_thread, this));
            return waiter.wait(can::State::ready, boost::posix_time::seconds(1));
        }
        return WrappedInterface::getState().isReady();
        #pragma GCC diagnostic pop
    }
    virtual bool init(const std::string &device, bool loopback, SettingsConstSharedPtr settings) override {
        if(!thread_ && WrappedInterface::init(device, loopback, settings)){
            StateWaiter waiter(this);
            thread_.reset(new boost::thread(&ThreadedInterface::run_thread, this));
            return waiter.wait(can::State::ready, boost::posix_time::seconds(1));
        }
        return WrappedInterface::getState().isReady();
    }

    virtual void shutdown(){
        WrappedInterface::shutdown();
        shutdown_internal();
    }
    void join(){
        if(thread_){
            thread_->join();
        }
    }
    virtual ~ThreadedInterface() {
        shutdown_internal();
    }
    ThreadedInterface(): WrappedInterface() {}
    template<typename T1> ThreadedInterface(const T1 &t1): WrappedInterface(t1) {}
    template<typename T1, typename T2> ThreadedInterface(const T1 &t1, const T2 &t2): WrappedInterface(t1, t2) {}

};


} // namespace can
#endif
