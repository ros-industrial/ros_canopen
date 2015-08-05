#include <class_loader/class_loader.h>
#include <canopen_master/master.h>

#include <set>

namespace canopen {
class SimpleSyncLayer: public SyncLayer {
    boost::shared_ptr<can::CommInterface> interface_;
    time_point read_time_, write_time_;
    boost::chrono::milliseconds step_, half_step_;

    std::set<void *> nodes_;
    boost::mutex nodes_mutex_;
    boost::atomic<size_t> nodes_size_;
protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            boost::this_thread::sleep_until(read_time_);
            write_time_ += step_;
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            can::Frame frame(properties.header_, 0);
            boost::this_thread::sleep_until(write_time_);
            if(nodes_size_){ //)
                interface_->send(frame);
            }
            read_time_ = get_abs_time(half_step_);
        }
    }

    virtual void handleInit(LayerStatus &status){
        write_time_ = get_abs_time(step_);
        read_time_ = get_abs_time(half_step_);
    }
    virtual void handleShutdown(LayerStatus &status) {
    }

    virtual void handleHalt(LayerStatus &status)  { /* nothing to do */ }
    virtual void handleDiag(LayerReport &report)  { /* TODO */ }
    virtual void handleRecover(LayerStatus &status)  { /* TODO */ }

public:
    SimpleSyncLayer(const SyncProperties &p, boost::shared_ptr<can::CommInterface> interface)
    : SyncLayer(p), interface_(interface), step_(p.period_ms_), half_step_(p.period_ms_/2), nodes_size_(0)
    {
    }

    virtual void addNode(void * const ptr) {
        boost::mutex::scoped_lock lock(nodes_mutex_);
        nodes_.insert(ptr);
        nodes_size_ = nodes_.size();
    }
    virtual void removeNode(void * const ptr)  {
        boost::mutex::scoped_lock lock(nodes_mutex_);
        nodes_.erase(ptr);
        nodes_size_ = nodes_.size();
    }
};
class SimpleMaster: public Master{
    boost::shared_ptr<can::CommInterface> interface_;
public:
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &properties){
        return boost::make_shared<SimpleSyncLayer>(properties, interface_);
    }
    SimpleMaster(boost::shared_ptr<can::CommInterface> interface) : interface_(interface)  {}

    class Allocator : public Master::Allocator{
    public:
        virtual boost::shared_ptr<Master> allocate(const std::string &name,  boost::shared_ptr<can::CommInterface> interface){
            return boost::make_shared<canopen::SimpleMaster>(interface);
        }
    };
};

}

boost::shared_ptr<canopen::Master> canopen::LocalMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::LocalMaster>(interface);
}
boost::shared_ptr<canopen::Master> canopen::SharedMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::SharedMaster>(name, interface);
}

boost::shared_ptr<canopen::Master> canopen::UnrestrictedMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::UnrestrictedMaster>(name, interface);
}

CLASS_LOADER_REGISTER_CLASS(canopen::LocalMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::SharedMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::UnrestrictedMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::SimpleMaster::Allocator, canopen::Master::Allocator);
