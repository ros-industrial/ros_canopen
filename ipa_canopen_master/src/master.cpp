#include <ipa_canopen_master/master.h>

namespace ipa_can{
std::size_t hash_value(ipa_can::Header const& h){ return (unsigned int)(h);}
}

using namespace ipa_canopen;

LocalSyncLayer::LocalSyncLayer(const SyncProperties &p, boost::shared_ptr<ipa_can::CommInterface> interface, bool loopback)
: SyncLayer(p), interface_(interface), msg_(p.header_,p.overflow_?1:0), loopback_(loopback)
{
}

void LocalSyncLayer::handleFrame(const ipa_can::Frame & msg){
    boost::mutex::scoped_lock lock(mutex_);
    if(track_timeout < timeout){
        track_timeout = timeout;
        LOG("MAX TIMEOUT: " << track_timeout.total_milliseconds());
    }
    timeout = boost::posix_time::seconds(0);
}

bool LocalSyncLayer::checkSync(){
    if(nodes_.empty()) return false;
    if(!loop_listener_) return true;

    bool okay = timeout == boost::posix_time::seconds(0);

    if(timeout > max_timeout){
        throw TimeoutException();
    }
    
    timeout += properties.period_;
    return okay;
}

bool LocalSyncLayer::sync() { 
    cond.notify_one();
    return true;
}

bool LocalSyncLayer::read() { 
    boost::system_time t = boost::get_system_time() + max_timeout;
    boost::mutex::scoped_lock lock(mutex_);
    return cond.timed_wait(lock,t);
}
bool LocalSyncLayer::write()  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!checkSync()) return true;
    
    if(properties.overflow_){
        ++msg_.data[0];
        if(msg_.data[0] > properties.overflow_) msg_.data[0] = 1;
    }
    
    interface_->send(msg_);
    return true;
}
bool LocalSyncLayer::report()  { return true; }

bool LocalSyncLayer::init() {
    boost::mutex::scoped_lock lock(mutex_);
    msg_.data[0] = 0;
    timeout = boost::posix_time::seconds(0);
    track_timeout = boost::posix_time::seconds(0);
    max_timeout =  properties.period_ + properties.period_;
    if(max_timeout < boost::posix_time::milliseconds(1000)){
        max_timeout = boost::posix_time::milliseconds(1000);
    }
    if(loopback_){
        loop_listener_ = interface_->createMsgListener(properties.header_, ipa_can::CommInterface::FrameDelegate(this,&LocalSyncLayer::handleFrame));
    }
    timer_.start(Timer::TimerDelegate(this, &LocalSyncLayer::sync), properties.period_);
    return true;
} 
bool LocalSyncLayer::recover() { return true; } 

bool LocalSyncLayer::shutdown() {
    boost::mutex::scoped_lock lock(mutex_);
    timer_.stop();
    loop_listener_.reset();
    return true;
}

boost::shared_ptr<SyncLayer> LocalMaster::getSync(const SyncProperties &p){
    boost::mutex::scoped_lock lock(mutex_);
    boost::unordered_map<ipa_can::Header, boost::shared_ptr<LocalSyncLayer> >::iterator it = layers_.find(p.header_);
    if(it == layers_.end()){
        
        std::pair<boost::unordered_map<ipa_can::Header, boost::shared_ptr<LocalSyncLayer> >::iterator, bool> res = layers_.insert(std::make_pair(p.header_, boost::make_shared<LocalSyncLayer>(p, interface_, loopback_)));
        it = res.first;
    }
    return it->second;
}


