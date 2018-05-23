
#ifndef CANOPEN_MOTOR_NODE_CONTROLLER_MANAGER_LAYER_H_
#define CANOPEN_MOTOR_NODE_CONTROLLER_MANAGER_LAYER_H_

#include <memory>

#include <ros/node_handle.h>
#include <atomic>
#include <canopen_master/canopen.h>
#include <canopen_motor_node/robot_layer.h>

// forward declarations
namespace controller_manager {
    class ControllerManager;
}

namespace canopen {

class ControllerManagerLayer : public canopen::Layer {
    std::shared_ptr<controller_manager::ControllerManager> cm_;
    canopen::RobotLayerSharedPtr robot_;
    ros::NodeHandle nh_;

    canopen::time_point last_time_;
    std::atomic<bool> recover_;
    const ros::Duration fixed_period_;

public:
    ControllerManagerLayer(const canopen::RobotLayerSharedPtr robot, const ros::NodeHandle &nh, const ros::Duration &fixed_period)
    :Layer("ControllerManager"), robot_(robot), nh_(nh), fixed_period_(fixed_period) {
    }

    virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleDiag(canopen::LayerReport &report) { /* nothing to do */ }
    virtual void handleHalt(canopen::LayerStatus &status) { /* nothing to do (?) */ }
    virtual void handleInit(canopen::LayerStatus &status);
    virtual void handleRecover(canopen::LayerStatus &status);
    virtual void handleShutdown(canopen::LayerStatus &status);
};

}  //  namespace canopen

#endif /* CANOPEN_MOTOR_NODE_CONTROLLER_MANAGER_LAYER_H_ */
