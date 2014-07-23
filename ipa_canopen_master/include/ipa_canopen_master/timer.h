#ifndef H_IPA_CANOPEN_TIMER
#define H_IPA_CANOPEN_TIMER

#include <ipa_can_interface/FastDelegate.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>

namespace ipa_canopen{

class Timer{
public:
    typedef fastdelegate::FastDelegate0<bool> TimerDelegate;
    Timer():work(io), timer(io),thread(fastdelegate::FastDelegate0<size_t>(&io, &boost::asio::io_service::run)){
    }
    
    void stop(){
        timer.cancel();
    }
    void start(const TimerDelegate &del, const boost::posix_time::time_duration &dur){
        stop();
        delegate = del;
        period = dur;
        restart();
    }
    void restart(){
        stop();
        timer.expires_from_now(period);
        timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
    }
    
private:
    boost::asio::io_service io;
    boost::asio::io_service::work work;
    boost::asio::deadline_timer timer;
    boost::thread thread;
    boost::posix_time::time_duration period;
    
    TimerDelegate delegate;
    void handler(const boost::system::error_code& ec){
        if(!ec){
            if(delegate && delegate()){
                timer.expires_at(timer.expires_at() + period);
                timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
            }
            
        }
    }    
};
    
}

#endif
