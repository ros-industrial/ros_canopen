
#include <canopen_motor_node/controller_manager_layer.h>
#include <controller_manager/controller_manager.h>

using namespace canopen;

void ControllerManagerLayer::handleRead(canopen::LayerStatus &status, const LayerState &current_state) {
    if(current_state > Shutdown){
        if(!cm_) status.error("controller_manager is not intialized");
    }
}

void ControllerManagerLayer::handleWrite(canopen::LayerStatus &status, const LayerState &current_state) {
    if(current_state > Shutdown){
        if(!cm_){
            status.error("controller_manager is not intialized");
        }else{
            time_point abs_now = canopen::get_abs_time();
            ros::Time now = ros::Time::now();

            ros::Duration period = fixed_period_;

            if(period.isZero()) {
                period.fromSec(boost::chrono::duration<double>(abs_now -last_time_).count());
            }

            last_time_ = abs_now;

            bool recover = recover_.exchange(false);
            cm_->update(now, period, recover);
            robot_->enforce(period, recover);
        }
    }
}

void ControllerManagerLayer::handleInit(canopen::LayerStatus &status) {
    if(cm_){
        status.warn("controller_manager is already intialized");
    }else{
        recover_ = true;
        last_time_ = canopen::get_abs_time();
        cm_.reset(new controller_manager::ControllerManager(robot_.get(), nh_));
    }
}

void ControllerManagerLayer::handleRecover(canopen::LayerStatus &status) {
    if(!cm_) status.error("controller_manager is not intialized");
    else recover_ = true;
}

void ControllerManagerLayer::handleShutdown(canopen::LayerStatus &status) {
    cm_.reset();
}

