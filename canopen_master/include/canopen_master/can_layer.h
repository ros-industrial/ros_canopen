#ifndef H_CAN_LAYER
#define H_CAN_LAYER

#include <socketcan_interface/threading.h>
#include "layer.h"

namespace canopen{

class CANLayer: public Layer{
    boost::mutex mutex_;
    boost::shared_ptr<can::DriverInterface> driver_;
    const std::string device_;
    const bool loopback_;
    can::Frame last_error_;
    can::CommInterface::FrameListener::Ptr error_listener_;
    void handleFrame(const can::Frame & msg){
        boost::mutex::scoped_lock lock(mutex_);
        last_error_ = msg;
        LOG("ID: " << msg.id);
    }
    boost::shared_ptr<boost::thread> thread_;

public:
    CANLayer(const boost::shared_ptr<can::DriverInterface> &driver, const std::string &device, bool loopback)
    : Layer(device + " Layer"), driver_(driver), device_(device), loopback_(loopback) { assert(driver_); }

    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            if(!driver_->getState().isReady()) status.error("CAN not ready");
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            if(!driver_->getState().isReady()) status.error("CAN not ready");
        }
    }

    virtual void handleDiag(LayerReport &report){
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
    
    virtual void handleInit(LayerStatus &status){
	if(thread_){
            status.warn("CAN thread already running");
        } else if(!driver_->init(device_, loopback_)) {
            status.error("CAN init failed");
        } else {
            thread_.reset(new boost::thread(&can::DriverInterface::run, driver_));
            error_listener_ = driver_->createMsgListener(can::ErrorHeader(), can::CommInterface::FrameDelegate(this, &CANLayer::handleFrame));
	    
	    if(!can::StateWaiter::wait_for(can::State::ready, driver_.get(), boost::posix_time::seconds(1))){
		status.error("CAN init timed out");
	    }
        }
	if(!driver_->getState().isReady()){
	  status.error("CAN is not ready");
	}
    }
    virtual void handleShutdown(LayerStatus &status){
        error_listener_.reset();
        driver_->shutdown();
        if(thread_){
            thread_->interrupt();
            thread_->join();
            thread_.reset();
        }
    }

    virtual void handleHalt(LayerStatus &status) { /* nothing to do */ }
    
    virtual void handleRecover(LayerStatus &status){
        if(!driver_->recover()) status.error("driver recover failed"); // TODO: implement logging for driver
    }

};
} // namespace canopen

#endif
