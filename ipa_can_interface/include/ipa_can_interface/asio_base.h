#ifndef H_IPA_ASIO_BASE
#define H_IPA_ASIO_BASE

#include <ipa_can_interface/interface.h>
#include <boost/asio.hpp>

namespace ipa_can{
    
template<typename FrameDelegate, typename StateDelegate, typename Socket> class AsioDriver{

    State state_;
    boost::mutex state_mutex_;
    boost::mutex send_mutex_;
    
    FrameDelegate frame_delegate_;
     StateDelegate state_delegate_;
protected:
    boost::asio::io_service io_service_;
    Socket socket_;
    Frame input_;
    
    virtual void triggerReadSome() = 0;
    virtual bool enqueue(const Frame & msg) = 0;
    
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
            frame_delegate_(input_);
            triggerReadSome();
        }else{
            setErrorCode(error);
        }
    }

    AsioDriver(FrameDelegate frame_delegate, StateDelegate state_delegate)
    : socket_(io_service_), frame_delegate_(frame_delegate), state_delegate_(state_delegate)
    {}
    
public:
    State getState(){
        boost::mutex::scoped_lock lock(state_mutex_);
        return state_;
    }
    void run(){
        setDriverState(socket_.is_open()?State::open : State::closed);
        
        if(getState().driver_state == State::open){
            boost::asio::io_service::work work(io_service_);
            setDriverState(State::ready);
            
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
    
    void shutdown(){
        if(socket_.is_open()) socket_.close();
        io_service_.stop();
        io_service_.reset();
    }
};


}; // namespace ipa_can
#endif
