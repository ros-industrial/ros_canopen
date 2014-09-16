#include <ipa_canopen_master/master.h>

namespace ipa_can{
std::size_t hash_value(ipa_can::Header const& h){ return (unsigned int)(h);}
}

using namespace ipa_canopen;

void IPCSyncMaster::run() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock = sync_obj_->waiter.get_lock();
    boost::posix_time::ptime abs_time = boost::get_system_time();
    
    ipa_can::Frame frame(sync_obj_->properties.header_, sync_obj_->properties.overflow_ ? 1 : 0);
    while(true){
        abs_time += sync_obj_->properties.period_;
        if(!sync_obj_->waiter.sync(boost::posix_time::seconds(1))) break; // TODO: handle error
        
        if(sync_obj_->nextSync(frame.data[0]))
            interface_->send(frame);

        boost::this_thread::sleep(abs_time);
        
    }
}


void IPCSyncLayer::init(LayerStatusExtended &status) {
    boost::mutex::scoped_lock lock(mutex_);
    if(!nodes_.empty()){
        status.set(LayerStatus::WARN);
        status.reason("Nodes list was not empty");
        nodes_.clear();
    }
    
    sync_master_->start(status);
}

boost::shared_ptr<SyncLayer> LocalMaster::getSync(const SyncProperties &p){
    boost::mutex::scoped_lock lock(mutex_);
    boost::unordered_map<ipa_can::Header, boost::shared_ptr<LocalIPCSyncMaster> >::iterator it = syncmasters_.find(p.header_);
    if(it == syncmasters_.end()){
        std::pair<boost::unordered_map<ipa_can::Header, boost::shared_ptr<LocalIPCSyncMaster> >::iterator, bool>
            res = syncmasters_.insert(std::make_pair(p.header_, boost::make_shared<LocalIPCSyncMaster>(p,interface_)));
        it = res.first;
    }else if(!it->second->matches(p)) return boost::shared_ptr<SyncLayer>();
    return boost::make_shared<IPCSyncLayer>(p, interface_, it->second);
}

