#ifndef H_IPA_CANOPEN_TIMER
#define H_IPA_CANOPEN_TIMER

#include <ipa_can_interface/FastDelegate.h>
#include <boost/asio.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/high_resolution_timer.hpp>

namespace ipa_canopen{

typedef boost::chrono::high_resolution_clock::time_point time_point;
typedef boost::chrono::high_resolution_clock::duration time_duration;
inline time_point get_abs_time(const time_duration& timeout) { return boost::chrono::high_resolution_clock::now() + timeout; }
    
    
class Timer{
public:
    typedef fastdelegate::FastDelegate0<bool> TimerDelegate;
    Timer():work(io), timer(io),thread(fastdelegate::FastDelegate0<size_t>(&io, &boost::asio::io_service::run)){
    }
    
    void stop(){
        boost::mutex::scoped_lock lock(mutex);
        timer.cancel();
    }
    void start(const TimerDelegate &del, const time_duration &dur){
        boost::mutex::scoped_lock lock(mutex);
        delegate = del;
        period = dur;
        timer.expires_from_now(period);
        timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
    }
    void restart(){
        boost::mutex::scoped_lock lock(mutex);
        timer.expires_from_now(period);
        timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
    }
    const time_duration & getPeriod(){
        boost::mutex::scoped_lock lock(mutex);
        return period;
    }
    
private:
    boost::asio::io_service io;
    boost::asio::io_service::work work;
    boost::asio::high_resolution_timer timer;
    time_duration period;
    boost::mutex mutex;
    boost::thread thread;
    
    TimerDelegate delegate;
    void handler(const boost::system::error_code& ec){
        if(!ec){
            boost::mutex::scoped_lock lock(mutex);
            if(delegate && delegate()){
                timer.expires_at(timer.expires_at() + period);
                timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
            }
            
        }
    }    
};
    
}

#endif
