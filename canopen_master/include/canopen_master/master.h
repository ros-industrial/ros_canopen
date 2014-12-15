#ifndef H_CANOPEN_MASTER
#define H_CANOPEN_MASTER

#include <canopen_master/canopen.h>

#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace canopen{

class SyncLayer: public Layer, public SyncCounter{
public:
    SyncLayer(const SyncProperties &p) : Layer("Sync layer"), SyncCounter(p) {}
};

class Master: boost::noncopyable{
public:
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &properties) = 0;
    virtual ~Master() {}
};

class IPCSyncWaiter{
    typedef boost::interprocess::interprocess_mutex interprocess_mutex;
    typedef boost::interprocess::interprocess_condition interprocess_condition;
    typedef boost::interprocess:: scoped_lock<interprocess_mutex> scoped_mutex_lock;
    
    interprocess_mutex master_mutex;

    interprocess_mutex sync_mutex;

    interprocess_mutex started_mutex;
    interprocess_condition started_cond;
    size_t sync_started;

    interprocess_mutex number_mutex;
    interprocess_condition number_cond;
    size_t number;

    template <typename AT> bool add(const AT &abs_time){
        scoped_mutex_lock sync_lock(sync_mutex, abs_time);
        if(!sync_lock) return false;
        // assume add and del share a common thread
        // scoped_mutex_lock cond_lock(number_mutex, abs_time);
        // if(!cond_lock) return false;
        ++number;
        return true;
    }
    template <typename AT> bool wait_started(const AT &abs_time){
        scoped_mutex_lock lock_started(started_mutex, abs_time);
        if(!lock_started) return false;

        while(sync_started == 0)
            if(!started_cond.timed_wait(lock_started, abs_time))
                return false;

        --sync_started;
        return true;
    }
    template <typename AT> bool start_sync(const AT &abs_time){
        {
            scoped_mutex_lock lock_started(started_mutex, abs_time);
            if(!lock_started) return false;
            sync_started = number; // no additional lock needed for number
        }
        started_cond.notify_all();
        return true;
    }
    template <typename AT> bool done_one(const AT &abs_time){
        scoped_mutex_lock cond_lock(number_mutex, abs_time);
        if(!cond_lock || number == 0) return false;
        if(--number == 0){
            cond_lock.unlock();
            number_cond.notify_all();
        }
        return true;
    }
    template <typename AT> bool wait_done(const AT &abs_time){
        scoped_mutex_lock number_lock(number_mutex, abs_time);
        if(!number_lock) return false;
        while(number != 0 && number_cond.timed_wait(number_lock, abs_time)) {}
        return number == 0;
    }
public:
    template <typename DT> bool wait(const DT &d){
        boost::posix_time::ptime  abs_time = boost::get_system_time() + d;
        return add(abs_time) && wait_started(abs_time);
    }
    template <typename DT> bool done(const DT &d){
        boost::posix_time::ptime  abs_time = boost::get_system_time() + d;
        return done_one(abs_time);
    }
    template <typename DT> bool sync(const DT &d){
        boost::posix_time::ptime  abs_time = boost::get_system_time() + d;
        scoped_mutex_lock lock_sync(sync_mutex, abs_time);
        return lock_sync && start_sync(abs_time) && wait_done(abs_time);
    }
    scoped_mutex_lock get_lock(){
        return scoped_mutex_lock(master_mutex); // well-behaved compilers won't copy (RVO)
    }
    IPCSyncWaiter() : sync_started(0), number(0) {}
};


class IPCSyncMaster {
public:
    class SyncObject{
        size_t sync_listeners;
        uint8_t last_sync;
        boost::interprocess::interprocess_mutex mutex;
    public:
        void enableSync(){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex);
            if(sync_listeners == 0) last_sync = 0; // reset sync counter
            ++sync_listeners;
        }
        void disableSync(){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex);
            if(sync_listeners) --sync_listeners;
        }
        bool nextSync( uint8_t & sync){
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex);
            if(sync_listeners == 0) return false;
            if(properties.overflow_){
                if(++last_sync > 240) last_sync = 1;
            }
            sync = last_sync;
            return true;
        }
        const SyncProperties properties;
        
        IPCSyncWaiter waiter;
        
        SyncObject(const SyncProperties &p) : sync_listeners(0), last_sync(0), properties(p) {}
    };
    IPCSyncMaster(boost::shared_ptr<can::CommInterface> interface)
    : interface_(interface), sync_obj_(0)
    {
    }
    void start(LayerStatus &status){
        if(thread_){
            status.warn("Sync thread already running");
            return;
        }
        sync_obj_ = getSyncObject(status);
        if(sync_obj_){
            thread_.reset(new boost::thread(&IPCSyncMaster::run, this));
        }else status.error("Sync object not found");
        
    }
    void stop(LayerStatus &status){
        if(!thread_){
            status.error();
            return;
        }
        thread_->interrupt();
        thread_->join();
        thread_.reset();
        sync_obj_ = 0; // locked externally
    }
    
    bool enableSync(){
        if(sync_obj_) sync_obj_->enableSync();
        return sync_obj_ != 0;
    }
    bool disableSync(){
        if(sync_obj_) sync_obj_->disableSync();
        return sync_obj_ != 0;
    }
    void wait(LayerStatus &status){
        if(sync_obj_){
            bool ok = sync_obj_->waiter.wait(sync_obj_->properties.period_);
            if(!ok) status.warn();
        }else status.error();
    }
    void notify(LayerStatus &status){
        if(sync_obj_){
            bool ok = sync_obj_->waiter.done(sync_obj_->properties.period_);
            if(!ok) status.warn();
        }else status.error();
    }

private:
    virtual SyncObject * getSyncObject(LayerStatus &status) = 0;
    
    boost::shared_ptr<boost::thread> thread_;
    boost::shared_ptr<can::CommInterface> interface_;
    
    void run();
    SyncObject * sync_obj_;
    
};


class IPCSyncLayer: public SyncLayer {
    can::CommInterface::FrameListener::Ptr sync_listener_;
    boost::shared_ptr<can::CommInterface> interface_;

    boost::shared_ptr<IPCSyncMaster> sync_master_;
    
    
    boost::mutex mutex_;
    boost::unordered_set<void const *> nodes_;
    
    uint8_t last_sync_;
    boost::condition_variable sync_cond_;
    boost::mutex sync_mutex_;

    void handleFrame(const can::Frame & msg){
        boost::mutex::scoped_lock lock(sync_mutex_);
        last_sync_ = msg.dlc == 1 ?  msg.data[0] : 1;
        sync_cond_.notify_all();
        
    }
    bool waitSync(){
        boost::posix_time::ptime  abs_time = boost::get_system_time() + properties.period_;
        boost::mutex::scoped_lock lock(sync_mutex_);
        while(last_sync_ == 0){
            if(!sync_cond_.timed_wait(lock, abs_time)) return false;
        }
        return true;
    }
    
public:
    IPCSyncLayer(const SyncProperties &p, boost::shared_ptr<can::CommInterface> interface, boost::shared_ptr<IPCSyncMaster> sync_master) 
    : SyncLayer(p), interface_(interface), sync_master_(sync_master)
    {
    }
    virtual void read(LayerStatus &status) {
        boost::mutex::scoped_lock lock(mutex_);
        if(!nodes_.empty()){
            if(!waitSync()){
                status.warn();
            }
        }
        sync_master_->wait(status);
        
    }
    virtual void write(LayerStatus &status) {
        boost::mutex::scoped_lock lock(mutex_);
        {
            boost::mutex::scoped_lock lock(sync_mutex_);
            last_sync_ = 0;
        }
        sync_master_->notify(status);
    }
    
    virtual void diag(LayerReport &report) {}
    virtual void init(LayerStatus &status);
    virtual void shutdown(LayerStatus &status) {
        boost::mutex::scoped_lock lock(mutex_);
        
        if(!nodes_.empty()){
            sync_master_->disableSync();
        }
        sync_listener_.reset();
        nodes_.clear();
        sync_master_->stop(status);
    }
    
    virtual void halt(LayerStatus &status) {}
    virtual void recover(LayerStatus &status) {}
    
    virtual void addNode(void * const ptr) {
        boost::mutex::scoped_lock lock(mutex_);
        bool was_empty = nodes_.empty();
        nodes_.insert(ptr);
        if(!nodes_.empty() && was_empty){
            sync_master_->enableSync();
        }
    }
    virtual void removeNode(void * const ptr)  { 
        boost::mutex::scoped_lock lock(mutex_);
        bool was_empty = nodes_.empty();
        nodes_.erase(ptr);
        
        if(nodes_.empty() && !was_empty){
            sync_master_->disableSync();
        }
    }
};

class LocalIPCSyncMaster : public IPCSyncMaster{
    SyncObject sync_obj_;
    virtual SyncObject * getSyncObject(LayerStatus &status) { return &sync_obj_; }
public:
    bool matches(const SyncProperties &p) const { return p == sync_obj_.properties; }
    LocalIPCSyncMaster(const SyncProperties &properties, boost::shared_ptr<can::CommInterface> interface)
    : IPCSyncMaster(interface), sync_obj_(properties)  { }
};

class LocalMaster: public Master{
    boost::mutex mutex_;
    boost::unordered_map<can::Header, boost::shared_ptr<LocalIPCSyncMaster> > syncmasters_;
    boost::shared_ptr<can::CommInterface> interface_;
public:
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &properties);
    LocalMaster(const std::string &name, boost::shared_ptr<can::CommInterface> interface) : interface_(interface)  {}
};

class SharedIPCSyncMaster : public IPCSyncMaster{
    boost::interprocess::managed_shared_memory &managed_shm_;
    const SyncProperties properties_;
    virtual SyncObject * getSyncObject(LayerStatus &status);
public:
    bool matches(const SyncProperties &p) const { return p == properties_; }
    SharedIPCSyncMaster(boost::interprocess::managed_shared_memory &managed_shm, const SyncProperties &properties, boost::shared_ptr<can::CommInterface> interface)
    : IPCSyncMaster(interface), managed_shm_(managed_shm), properties_(properties)  { }
};

class SharedMaster: public Master{
    // TODO: test multi chain 
    const std::string name_;
    struct Remover{
        const std::string name;
        Remover(const std::string &n) : name(n) {  boost::interprocess::shared_memory_object::remove(name.c_str()); }
        ~Remover(){  boost::interprocess::shared_memory_object::remove(name.c_str()); }
    } remover_;
    boost::interprocess::managed_shared_memory managed_shm_;
    boost::mutex mutex_;
    boost::unordered_map<can::Header, boost::shared_ptr<SharedIPCSyncMaster> > syncmasters_;
    boost::shared_ptr<can::CommInterface> interface_;
public:
    SharedMaster(const std::string &name, boost::shared_ptr<can::CommInterface> interface)
    : name_("canopen_master_shm_"+name), remover_(name_.c_str()),
        managed_shm_(boost::interprocess::open_or_create, name_.c_str(), 4096),
        interface_(interface)  {}
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &properties);
};

} // canopen
#endif // !H_CANOPEN_MASTER
