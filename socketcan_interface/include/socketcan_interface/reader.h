#ifndef H_CAN_BUFFERED_READER
#define H_CAN_BUFFERED_READER

#include <socketcan_interface/interface.h>
#include <deque>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace can{

class BufferedReader {
    std::deque<can::Frame> buffer_;
    boost::mutex mutex_;
    boost::condition_variable cond_;
    CommInterface::FrameListener::Ptr listener_;
    bool enabled_;

    void handleFrame(const can::Frame & msg){
        boost::mutex::scoped_lock lock(mutex_);
        if(enabled_){
            buffer_.push_back(msg);
            cond_.notify_one();
        }else{
            LOG("discarded message " /*<< tostring(msg)*/); // enable message printing
        }
    }
public:
    class ScopedEnabler{
        BufferedReader &reader_;
    public:
        ScopedEnabler(BufferedReader &reader) : reader_(reader) { reader_.enable(); }
        ~ScopedEnabler() { reader_.disable(); }
    };

    BufferedReader() : enabled_(true) {}
    BufferedReader(bool enable) : enabled_(enable) {}

    BufferedReader(boost::shared_ptr<CommInterface> interface, const Frame::Header& h){
        listen(interface,h);
    }
    BufferedReader(bool enable, boost::shared_ptr<CommInterface> interface, const Frame::Header& h) : enabled_(enable) {
        listen(interface,h);
    }

    void enable(){
        boost::mutex::scoped_lock lock(mutex_);
        enabled_ = true;
    }

    void disable(){
        boost::mutex::scoped_lock lock(mutex_);
        enabled_ = false;
    }

    void listen(boost::shared_ptr<CommInterface> interface, const Frame::Header& h){
        boost::mutex::scoped_lock lock(mutex_);
        listener_ = interface->createMsgListener(h, CommInterface::FrameDelegate(this, &BufferedReader::handleFrame));
        buffer_.clear();
    }

    template<typename DurationType> bool read(can::Frame * msg, const DurationType &duration){
        boost::mutex::scoped_lock lock(mutex_);
        boost::chrono::high_resolution_clock::time_point abs_time =  boost::chrono::high_resolution_clock::now() + duration;

        while(buffer_.empty() && cond_.wait_until(lock,abs_time)  != boost::cv_status::timeout)
        {}

        if(buffer_.empty()){
            return false;
        }

        if(msg){
            *msg = buffer_.front();
            buffer_.pop_front();
        }
        return true;

    }

};

} // namespace can
#endif
