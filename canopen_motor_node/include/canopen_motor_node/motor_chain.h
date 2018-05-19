
#ifndef CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_
#define CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_

#include <memory>

#include <ros/node_handle.h>
#include <canopen_chain_node/ros_chain.h>

#include <canopen_motor_node/robot_layer.h>
#include <canopen_motor_node/controller_manager_layer.h>


namespace canopen {

class MotorChain : public canopen::RosChain {
    ClassAllocator<canopen::MotorBase> motor_allocator_;
    std::shared_ptr< canopen::LayerGroupNoDiag<canopen::MotorBase> > motors_;
    RobotLayerSharedPtr robot_layer_;

    std::shared_ptr<ControllerManagerLayer> cm_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const canopen::NodeSharedPtr &node, const LoggerSharedPtr &logger);

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);

    virtual bool setup_chain();
};

}  // namespace canopen

#endif /* INCLUDE_CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_ */
