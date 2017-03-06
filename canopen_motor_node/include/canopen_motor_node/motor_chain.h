
#ifndef CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_
#define CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_

#include <ros/node_handle.h>
#include <boost/shared_ptr.hpp>
#include <canopen_chain_node/ros_chain.h>

#include <canopen_motor_node/robot_layer.h>
#include <canopen_motor_node/controller_manager_layer.h>


namespace canopen {

class MotorChain : public canopen::RosChain {
    ClassAllocator<canopen::MotorBase> motor_allocator_;
    boost::shared_ptr< canopen::LayerGroupNoDiag<canopen::MotorBase> > motors_;
    boost::shared_ptr<RobotLayer> robot_layer_;

    boost::shared_ptr<ControllerManagerLayer> cm_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<Node> &node, const boost::shared_ptr<Logger> &logger);

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv);

    virtual bool setup_chain();
};

}  // namespace canopen

#endif /* INCLUDE_CANOPEN_MOTOR_NODE_MOTOR_CHAIN_H_ */
