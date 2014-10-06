#ifndef H_IPA_CAN_LAYER
#define H_IPA_CAN_LAYER

#include "layer.h"

namespace ipa_canopen{
    
template<typename Driver> class CANLayer: public Layer{
    boost::mutex mutex_;
    boost::shared_ptr<Driver> driver_;
    const std::string device_;
    const unsigned int bitrate_;
    ipa_can::Frame last_error_;
    ipa_can::CommInterface::FrameListener::Ptr error_listener_;
    void handleFrame(const ipa_can::Frame & msg){
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

    virtual void report(LayerStatusExtended &status){
        ipa_can::State s = driver_->getState();
        if(!s.isReady()){
            status.error("CAN layer not ready");
            status.add("driver_state", int(s.driver_state));
        }
        if(s.error_code){
            status.add("socket_error", s.error_code);
        }
        if(s.internal_error != 0){
            status.add("internal_error", int(s.internal_error));
            std::string desc;
            if(driver_->translateError(s.internal_error, desc)) status.add("internal_error_desc", desc);
            std::stringstream sstr;
            sstr << std::hex;
            {
                boost::mutex::scoped_lock lock(mutex_);
                for(size_t i=0; i < last_error_.dlc; ++i){
                    sstr << (unsigned int) last_error_.data[i] << " ";
                }
            }
            status.add("can_error_frame", sstr.str());

        }

    }
    
    virtual void init(LayerStatusExtended &status){
        if(!driver_->init(device_, bitrate_)){
            status.error("CAN init failed");
        }else{
            error_listener_ = driver_->createMsgListener(ipa_can::ErrorHeader(), ipa_can::CommInterface::FrameDelegate(this, &CANLayer::handleFrame));
        }
    }
    virtual void shutdown(LayerStatus &status){
        error_listener_.reset();
        driver_->shutdown();
    }

    virtual void halt(LayerStatus &status) { /* nothing to do */ }
    
    virtual void recover(LayerStatusExtended &status){
        if(!driver_->recover()) status.error("driver recover failed"); // TODO: implement logging for driver
    }

};
} // namespace ipa_canopen

#endif
