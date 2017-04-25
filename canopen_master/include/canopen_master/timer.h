#ifndef H_CANOPEN_TIMER
#define H_CANOPEN_TIMER

#include <socketcan_interface/FastDelegate.h>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/high_resolution_timer.hpp>

namespace canopen{

class Timer{
public:
    typedef fastdelegate::FastDelegate0<bool> TimerDelegate;
    Timer():work(io), timer(io),thread(fastdelegate::FastDelegate0<size_t>(&io, &boost::asio::io_service::run)){
    }
    
    void stop(){
        boost::mutex::scoped_lock lock(mutex);
        timer.cancel();
    }
    template<typename T> void start(const TimerDelegate &del, const  T &dur, bool start_now = true){
        boost::mutex::scoped_lock lock(mutex);
        delegate = del;
        period = boost::chrono::duration_cast<boost::chrono::high_resolution_clock::duration>(dur);
        if(start_now){
            timer.expires_from_now(period);
            timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
        }
    }
    void restart(){
        boost::mutex::scoped_lock lock(mutex);
        timer.expires_from_now(period);
        timer.async_wait(fastdelegate::FastDelegate1<const boost::system::error_code&>(this, &Timer::handler));
    }
    const  boost::chrono::high_resolution_clock::duration & getPeriod(){
        boost::mutex::scoped_lock lock(mutex);
        return period;
    }
    ~Timer(){
        io.stop();
        thread.join();
    }
    
private:
    boost::asio::io_service io;
    boost::asio::io_service::work work;
    boost::asio::basic_waitable_timer<boost::chrono::high_resolution_clock> timer;
    boost::chrono::high_resolution_clock::duration period;
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
