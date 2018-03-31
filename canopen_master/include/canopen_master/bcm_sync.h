#ifndef H_BCM_SYNC
#define H_BCM_SYNC

#include <socketcan_interface/bcm.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_master/canopen.h>

namespace canopen {

template<typename T > std::string join(const T &container, const std::string &delim){
    if(container.empty()) return std::string();
    std::stringstream sstr;
    typename T::const_iterator it = container.begin();
    sstr << *it;
    for(++it; it != container.end(); ++it){
        sstr << delim << *it;
    }
    return sstr.str();
}

class BCMsync : public Layer {
    boost::mutex mutex_;

    std::string device_;

    std::set<int> ignored_nodes_;
    std::set<int> monitored_nodes_;
    std::set<int> known_nodes_;
    std::set<int> started_nodes_;

    bool sync_running_;
    can::BCMsocket bcm_;
    can::SocketCANDriverSharedPtr  driver_;
    uint16_t sync_ms_;
    can::FrameListenerConstSharedPtr handler_;

    std::vector<can::Frame> sync_frames_;

    bool skipNode(uint8_t id){
        if(ignored_nodes_.find(id) != ignored_nodes_.end()) return true;
        if(!monitored_nodes_.empty() && monitored_nodes_.find(id) == monitored_nodes_.end()) return true;
        known_nodes_.insert(id);
        return false;
    }

    void handleFrame(const can::Frame &frame){
        boost::mutex::scoped_lock lock(mutex_);

        if(frame.id == NMT_ID){
            if(frame.dlc > 1){
                uint8_t cmd = frame.data[0];
                uint8_t id = frame.data[1];
                if(skipNode(id)) return;

                if(cmd == 1){ // started
                    if(id != 0) started_nodes_.insert(id);
                    else started_nodes_.insert(known_nodes_.begin(), known_nodes_.end()); // start all
                }else{
                    if(id != 0) started_nodes_.erase(id);
                    else started_nodes_.clear(); // stop all
                }
            }
        }else if((frame.id & ~ALL_NODES_MASK) == HEARTBEAT_ID){
            int id = frame.id & ALL_NODES_MASK;
            if(skipNode(id)) return;

            if(frame.dlc > 0 && frame.data[0] ==  canopen::Node::Stopped) started_nodes_.erase(id);
        }

        // toggle sync if needed
        if(started_nodes_.empty() && sync_running_){
            sync_running_ = !bcm_.stopTX(sync_frames_.front());
        }else if(!started_nodes_.empty() && !sync_running_){
            sync_running_ = bcm_.startTX(boost::chrono::milliseconds(sync_ms_),sync_frames_.front(), sync_frames_.size(), &sync_frames_[0]);
        }
    }
protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
        }
    }
    virtual void handleDiag(LayerReport &report){
        boost::mutex::scoped_lock lock(mutex_);
        if(!sync_running_) report.warn("sync is not running");

        report.add("sync_running", sync_running_);
        report.add("known_nodes", join(known_nodes_, ", "));
        report.add("started_nodes", join(started_nodes_, ", "));
    }

    virtual void handleInit(LayerStatus &status){
        boost::mutex::scoped_lock lock(mutex_);
        started_nodes_.clear();

        if(!bcm_.init(device_)){
            status.error("BCM_init failed");
            return;
        }
        int sc = driver_->getInternalSocket();

        struct can_filter filter[2];

        filter[0].can_id   = NMT_ID;
        filter[0].can_mask = CAN_SFF_MASK;
        filter[1].can_id   = HEARTBEAT_ID;
        filter[1].can_mask =  ~ALL_NODES_MASK;

        if(setsockopt(sc, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0){
            status.warn("could not set filter");
            return;
        }

        handler_ = driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &BCMsync::handleFrame));
    }
    virtual void handleShutdown(LayerStatus &status){
        boost::mutex::scoped_lock lock(mutex_);
        handler_.reset();
        bcm_.shutdown();
    }

    virtual void handleHalt(LayerStatus &status) {
        boost::mutex::scoped_lock lock(mutex_);
        if(sync_running_){
            bcm_.stopTX(sync_frames_.front());
            sync_running_ = false;
            started_nodes_.clear();
        }
    }

    virtual void handleRecover(LayerStatus &status){
        handleShutdown(status);
        handleInit(status);
    }
public:
    static const uint32_t ALL_NODES_MASK = 127;
    static const uint32_t HEARTBEAT_ID = 0x700;
    static const uint32_t NMT_ID = 0x000;

    BCMsync(const std::string &device, can::SocketCANDriverSharedPtr  driver, const SyncProperties &sync_properties)
    : Layer(device + " SyncLayer"), device_(device), sync_running_(false), sync_ms_(sync_properties.period_ms_), driver_(driver) {
        if(sync_properties.overflow_ == 0){
            sync_frames_.resize(1);
            sync_frames_[0] = can::Frame(sync_properties.header_,0);
        }else{
            sync_frames_.resize(sync_properties.overflow_);
            for(int i = 0; i < sync_properties.overflow_; ++i){
                sync_frames_[i] = can::Frame(sync_properties.header_,1);
                sync_frames_[i].data[0] = i+1;
            }
        }
    }
    template <class T> void setMonitored(const T &other){
        monitored_nodes_.clear();
        monitored_nodes_.insert(other.begin(), other.end());
    }
    template <class T> void setIgnored(const T &other){
        ignored_nodes_.clear();
        ignored_nodes_.insert(other.begin(), other.end());
    }
};

}
#endif
