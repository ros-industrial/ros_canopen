#ifndef H_CAN_LAYER
#define H_CAN_LAYER

#include "layer.h"

namespace canopen{
    
template<typename Driver> class CANLayer: public Layer{
    boost::mutex mutex_;
    boost::shared_ptr<Driver> driver_;
    const std::string device_;
    const unsigned int bitrate_;
    can::Frame last_error_;
    can::CommInterface::FrameListener::Ptr error_listener_;
    void handleFrame(const can::Frame & msg){
        boost::mutex::scoped_lock lock(mutex_);
        last_error_ = msg;
        LOG("ID: " << msg.id);
    }

public:
    CANLayer(const boost::shared_ptr<Driver> &driver, const std::string &device, const unsigned int bitrate)
    : Layer(device + " Layer"), driver_(driver), device_(device), bitrate_(bitrate) { assert(driver_); }
    virtual void read(LayerStatus &status){
        if(!driver_->getState().isReady()) status.error();
    }
    virtual void write(LayerStatus &status){
        if(!driver_->getState().isReady()) status.error();
    }

    virtual void diag(LayerReport &report){
        can::State s = driver_->getState();
        if(!s.isReady()){
            report.error("CAN layer not ready");
            report.add("driver_state", int(s.driver_state));
        }
        if(s.error_code){
            report.add("socket_error", s.error_code);
        }
        if(s.internal_error != 0){
            report.add("internal_error", int(s.internal_error));
            std::string desc;
            if(driver_->translateError(s.internal_error, desc)) report.add("internal_error_desc", desc);
            std::stringstream sstr;
            sstr << std::hex;
            {
                boost::mutex::scoped_lock lock(mutex_);
                for(size_t i=0; i < last_error_.dlc; ++i){
                    sstr << (unsigned int) last_error_.data[i] << " ";
                }
            }
            report.add("can_error_frame", sstr.str());

        }

    }
    
    virtual void init(LayerStatus &status){
        if(!driver_->init(device_, bitrate_)){
            status.error("CAN init failed");
        }else{
            error_listener_ = driver_->createMsgListener(can::ErrorHeader(), can::CommInterface::FrameDelegate(this, &CANLayer::handleFrame));
        }
    }
    virtual void shutdown(LayerStatus &status){
        error_listener_.reset();
        driver_->shutdown();
    }

    virtual void halt(LayerStatus &status) { /* nothing to do */ }
    
    virtual void recover(LayerStatus &status){
        if(!driver_->recover()) status.error("driver recover failed"); // TODO: implement logging for driver
    }

};
} // namespace canopen

#endif
