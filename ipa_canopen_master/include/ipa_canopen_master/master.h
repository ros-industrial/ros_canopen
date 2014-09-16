#ifndef H_IPA_CANOPEN_MASTER
#define H_IPA_CANOPEN_MASTER

#include <ipa_canopen_master/canopen.h>

namespace ipa_canopen{

class SyncLayer: public SimpleLayer, public SyncCounter{ // TODO: implement Layer
public:
    SyncLayer(const SyncProperties &p) : SimpleLayer("Sync layer"), SyncCounter(p) {}
};

class Master: boost::noncopyable{
public:
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &p) = 0;
    virtual ~Master() {}
};
    
class LocalSyncLayer:  public SyncLayer{  // TODO: implement Layer
public:
    LocalSyncLayer(const SyncProperties &p, boost::shared_ptr<ipa_can::CommInterface> interface, bool loopback);
    
    virtual bool read();
    virtual bool write();
    virtual bool report();
    virtual bool init();
    virtual bool recover();
    virtual bool shutdown();
    
    virtual void addNode(void * const ptr) { boost::mutex::scoped_lock lock(mutex_); nodes_.insert(ptr); }
    virtual void removeNode(void * const ptr)  { boost::mutex::scoped_lock lock(mutex_); nodes_.erase(ptr); }
private:
    
    boost::mutex mutex_;
    boost::condition_variable cond;
    boost::unordered_set<void const *> nodes_;
    
    boost::shared_ptr<ipa_can::CommInterface> interface_;
    ipa_can::Frame msg_;
    bool loopback_;
    ipa_can::CommInterface::FrameListener::Ptr loop_listener_;

    boost::posix_time::time_duration timeout;
    boost::posix_time::time_duration max_timeout;
    boost::posix_time::time_duration track_timeout;
    
    Timer timer_;

    void handleFrame(const ipa_can::Frame & msg);

    bool checkSync();
    bool sync();
};

class LocalMaster: public Master{
    boost::mutex mutex_;
    boost::unordered_map<ipa_can::Header, boost::shared_ptr<LocalSyncLayer> > layers_;
    boost::shared_ptr<ipa_can::CommInterface> interface_;
    const bool loopback_;
public:
    boost::shared_ptr<SyncLayer> getSync(const SyncProperties &p);
    template<typename Driver> LocalMaster(boost::shared_ptr<Driver> interface): interface_(interface), loopback_(interface->doesLoopBack()) {}
};

} // ipa_canopen
#endif // !H_IPA_CANOPEN_MASTER
