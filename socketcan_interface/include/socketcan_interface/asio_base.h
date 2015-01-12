#ifndef H_ASIO_BASE
#define H_ASIO_BASE

#include <socketcan_interface/interface.h>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace can{


template<typename FrameDelegate, typename StateDelegate, typename Socket> class AsioDriver : public DriverInterface{
    static void call_delegate(const FrameDelegate &delegate, const Frame &msg){
        delegate(msg);
    }

    State state_;
    boost::mutex state_mutex_;
    boost::mutex socket_mutex_;
    
    FrameDelegate frame_delegate_;
    StateDelegate state_delegate_;
protected:
    boost::asio::io_service io_service_;
    Socket socket_;
    Frame input_;
    
    virtual void triggerReadSome() = 0;
    virtual bool enqueue(const Frame & msg) = 0;
    
    void dispatchFrame(const Frame &msg){
        io_service_.post(boost::bind(call_delegate, frame_delegate_,msg)); // copies msg
    }
    void setErrorCode(const boost::system::error_code& error){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.error_code != error){
            state_.error_code = error;
            state_delegate_(state_);
        }
    }
    void setInternalError(unsigned int internal_error){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.internal_error != internal_error){
            state_.internal_error = internal_error;
            state_delegate_(state_);
        }
    }

    void setDriverState(State::DriverState state){
        boost::mutex::scoped_lock lock(state_mutex_);
        if(state_.driver_state != state){
            state_.driver_state = state;
            state_delegate_(state_);
        }
    }
    
    void frameReceived(const boost::system::error_code& error){
        if(!error){
            dispatchFrame(input_);
            triggerReadSome();
        }else{
            setErrorCode(error);
        }
    }

    AsioDriver(FrameDelegate frame_delegate, StateDelegate state_delegate)
    : frame_delegate_(frame_delegate), state_delegate_(state_delegate), socket_(io_service_)
    {}

public:
    virtual ~AsioDriver() {}
    
    State getState(){
        boost::mutex::scoped_lock lock(state_mutex_);
        return state_;
    }
    virtual void run(){
        setDriverState(socket_.is_open()?State::open : State::closed);
        
        if(getState().driver_state == State::open){
            io_service_.reset();
            boost::asio::io_service::work work(io_service_);
            setDriverState(State::ready);

            boost::thread post_thread(boost::bind(&boost::asio::io_service::run, &io_service_));
            
            triggerReadSome();
            
            boost::system::error_code ec;
            io_service_.run(ec);
            setErrorCode(ec);
            
            setDriverState(socket_.is_open()?State::open : State::closed);
        }   
        state_delegate_(state_);
    }
    bool send(const Frame & msg){
        return getState().driver_state == State::ready && enqueue(msg);
    }
    
    virtual void shutdown(){
        LOG("SHUTDOWN");
        if(socket_.is_open()){
            socket_.cancel();
            socket_.close();
        }
        io_service_.stop();
    }
};

class StateWaiter{
    boost::mutex mutex_;
    boost::condition_variable cond_;
    can::StateInterface::StateListener::Ptr state_listener_;
    can::State state_;
    void updateState(const can::State &s){
        boost::mutex::scoped_lock lock(mutex_);
        state_ = s;
        lock.unlock();
        cond_.notify_one();
    }
public:
    template<typename InterfaceType> StateWaiter(InterfaceType *interface){
        state_ = interface->getState();
        state_listener_ = interface->createStateListener(can::StateInterface::StateDelegate(this, &StateWaiter::updateState));
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
    template<typename InterfaceType, typename DurationType> static bool wait_for(const can::State::DriverState &s, InterfaceType *interface, const DurationType &duration){
        StateWaiter waiter(interface);
        return waiter.wait(s,duration);
    }
};

template<typename WrappedInterface> class ThreadedInterface : public WrappedInterface{
    boost::shared_ptr<boost::thread> thread_;
    void run_thread(){
        WrappedInterface::run();
    }
public:
    virtual bool init(const std::string &device, unsigned int bitrate) {
        if(!thread_ && WrappedInterface::init(device, bitrate)){
            thread_.reset(new boost::thread(&ThreadedInterface::run_thread, this));
            return StateWaiter::wait_for(can::State::ready, this, boost::posix_time::seconds(1));
        }
        return WrappedInterface::getState().isReady();
    }
    virtual void shutdown(){
        WrappedInterface::shutdown();
        if(thread_){
            thread_->interrupt();
            thread_->join();
            thread_.reset();
        }
    }
    void join(){
        if(thread_){
            thread_->join();
        }
    }
    virtual ~ThreadedInterface() {}
    ThreadedInterface(): WrappedInterface() {}
    template<typename T1> ThreadedInterface(const T1 &t1): WrappedInterface(t1) {}
    template<typename T1, typename T2> ThreadedInterface(const T1 &t1, const T2 &t2): WrappedInterface(t1, t2) {}
    
};


} // namespace can
#endif
