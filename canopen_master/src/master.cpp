#include <canopen_master/master.h>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/list.hpp>

namespace can{
std::size_t hash_value(can::Header const& h){ return (unsigned int)(h);}
}

using namespace canopen;

void IPCSyncMaster::run() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock = sync_obj_->waiter.get_lock();
    boost::posix_time::ptime abs_time = boost::get_system_time();
    
    can::Frame frame(sync_obj_->properties.header_, sync_obj_->properties.overflow_ ? 1 : 0);
    while(true){
        abs_time += boost::posix_time::milliseconds(sync_obj_->properties.period_ms_);
        if(abs_time >= boost::get_system_time()){
            if(!sync_obj_->waiter.sync(abs_time)) LOG("Slave timeout");

            if(sync_obj_->nextSync(frame.data[0])){
                interface_->send(frame);
            }

            boost::this_thread::sleep(abs_time);
        }
        
    }
}


void IPCSyncLayer::handleInit(LayerStatus &status) {
    boost::mutex::scoped_lock lock(mutex_);
    if(!nodes_.empty()){
        status.warn("Nodes list was not empty");
        nodes_.clear();
    }
    sync_master_->start(status);
}

// TODO: unify/combine

boost::shared_ptr<SyncLayer> LocalMaster::getSync(const SyncProperties &p){
    boost::mutex::scoped_lock lock(mutex_);
    boost::unordered_map<can::Header, boost::shared_ptr<LocalIPCSyncMaster> >::iterator it = syncmasters_.find(p.header_);
    if(it == syncmasters_.end()){
        std::pair<boost::unordered_map<can::Header, boost::shared_ptr<LocalIPCSyncMaster> >::iterator, bool>
            res = syncmasters_.insert(std::make_pair(p.header_, boost::make_shared<LocalIPCSyncMaster>(p,interface_)));
        it = res.first;
    }else if(!it->second->matches(p)) return boost::shared_ptr<SyncLayer>();
    return boost::make_shared<IPCSyncLayer>(p, interface_, it->second);
}

boost::shared_ptr<SyncLayer> SharedMaster::getSync(const SyncProperties &p){
    boost::mutex::scoped_lock lock(mutex_);
    boost::unordered_map<can::Header, boost::shared_ptr<SharedIPCSyncMaster> >::iterator it = syncmasters_.find(p.header_);
    if(it == syncmasters_.end()){
        std::pair<boost::unordered_map<can::Header, boost::shared_ptr<SharedIPCSyncMaster> >::iterator, bool>
            res = syncmasters_.insert(std::make_pair(p.header_, boost::make_shared<SharedIPCSyncMaster>(boost::ref(managed_shm_), p,interface_)));
        it = res.first;
    }else if(!it->second->matches(p)) return boost::shared_ptr<SyncLayer>();
    return boost::make_shared<IPCSyncLayer>(p, interface_, it->second);
}
IPCSyncMaster::SyncObject * SharedIPCSyncMaster::getSyncObject(LayerStatus &status){
    typedef boost::interprocess::allocator<SyncObject, boost::interprocess::managed_shared_memory::segment_manager>  SyncAllocator;
    typedef boost::interprocess::list<SyncObject, SyncAllocator> SyncList;
    
    boost::interprocess::interprocess_mutex  *list_mutex = managed_shm_.find_or_construct<boost::interprocess::interprocess_mutex>("SyncListMutex")();
    
    if(!list_mutex){
        status.error("Could not find/construct SyncListMutex");
        return 0;
    }
    
    SyncList *synclist = managed_shm_.find_or_construct<SyncList>("SyncList")(managed_shm_.get_allocator<SyncAllocator>());
    
    if(!synclist){
        status.error("Could not find/construct SyncList");
        return 0;
    }
    
    
    SyncObject * sync_obj = 0;

    boost::posix_time::ptime abs_time = boost::get_system_time() + boost::posix_time::seconds(1);
        
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(*list_mutex, abs_time);
    if(!lock){
        status.error("Could not lock master mutex");
        return 0;
    }
    
    for(SyncList::iterator it = synclist->begin(); it != synclist->end(); ++it){
        if( it->properties.header_ == properties_.header_){
            
            if(it->properties.overflow_ != properties_.overflow_ || it->properties.period_ms_ != properties_.period_ms_){
                status.error("sync properties mismatch");
                return 0;
            }
            
            sync_obj = &(*it);
            break;
        }
    }
    if(!sync_obj) {
        synclist->emplace_back(properties_);
        sync_obj = &synclist->back();
    }
    return sync_obj;
}
