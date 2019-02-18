#ifndef H_CANOPEN_TIMER
#define H_CANOPEN_TIMER

#include <functional>
#include <memory>

#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/high_resolution_timer.hpp>

#include <socketcan_interface/delegates.h>

namespace canopen{

class Timer{
public:
    using TimerFunc = std::function<bool(void)>;
    using TimerDelegate [[deprecated("use TimerFunc instead")]] = can::DelegateHelper<TimerFunc>;

    Timer():work(io), timer(io),thread(std::bind(
        static_cast<size_t(boost::asio::io_service::*)(void)>(&boost::asio::io_service::run), &io))
    {
    }

    void stop(){
        boost::mutex::scoped_lock lock(mutex);
        timer.cancel();
    }
    template<typename T> void start(const TimerFunc &del, const  T &dur, bool start_now = true){
        boost::mutex::scoped_lock lock(mutex);
        delegate = del;
        period = boost::chrono::duration_cast<boost::chrono::high_resolution_clock::duration>(dur);
        if(start_now){
            timer.expires_from_now(period);
            timer.async_wait(std::bind(&Timer::handler, this, std::placeholders::_1));
        }
    }
    void restart(){
        boost::mutex::scoped_lock lock(mutex);
        timer.expires_from_now(period);
        timer.async_wait(std::bind(&Timer::handler, this, std::placeholders::_1));
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

    TimerFunc delegate;
    void handler(const boost::system::error_code& ec){
        if(!ec){
            boost::mutex::scoped_lock lock(mutex);
            if(delegate && delegate()){
                timer.expires_at(timer.expires_at() + period);
                timer.async_wait(std::bind(&Timer::handler, this, std::placeholders::_1));
            }

        }
    }
};

}

#endif
