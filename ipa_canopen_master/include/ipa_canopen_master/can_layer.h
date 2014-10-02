#ifndef H_IPA_CAN_LAYER
#define H_IPA_CAN_LAYER

#include "layer.h"

namespace ipa_canopen{
    
template<typename Driver> class CANLayer: public Layer{
    boost::shared_ptr<Driver> driver_;
    const std::string device_;
    const unsigned int bitrate_;
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
        // TODO: implement history
        ipa_can::State s = driver_->getState();
        if(!s.isReady()){
            status.error("CAN layer not ready");
            status.add("driver_state", int(s.driver_state));
        }
        if(s.error_code){
            status.add("socket_error", s.error_code);
        }
        if(s.internal_error != 0){
            // TODO: interprete error
            status.add("internal_error", int(s.internal_error));
            std::string desc;
            if(driver_->translateError(s.internal_error, desc)) status.add("internal_error_desc", desc);
        }

    }
    
    virtual void init(LayerStatusExtended &status){
        if(!driver_->init(device_, bitrate_)){
            status.error("CAN init failed");
        }
    }
    virtual void shutdown(LayerStatus &status){
        driver_->shutdown();
    }

    virtual void halt(LayerStatus &status) { /* nothing to do */ }
    
    virtual void recover(LayerStatusExtended &status){
        if(!driver_->recover()) status.error("driver recover failed"); // TODO: implement logging for driver
    }

};
} // namespace ipa_canopen

#endif
