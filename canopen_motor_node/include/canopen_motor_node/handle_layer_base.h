
#ifndef CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_
#define CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_

#include <string>
#include <canopen_master/layer.h>

namespace canopen {

class HandleLayerBase: public canopen::Layer{
public:
    HandleLayerBase(const std::string &name) : Layer(name) {}

    enum CanSwitchResult{
        NotSupported,
        NotReadyToSwitch,
        ReadyToSwitch,
        NoNeedToSwitch
    };

    virtual CanSwitchResult canSwitch(const canopen::MotorBase::OperationMode &m) = 0;
    virtual bool switchMode(const canopen::MotorBase::OperationMode &m) = 0;

    virtual bool forwardForMode(const canopen::MotorBase::OperationMode &m) = 0;

    virtual void registerHandle(hardware_interface::JointStateInterface &iface) = 0;
    virtual hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface,
                                                            const joint_limits_interface::JointLimits &limits,
                                                            const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;
    virtual hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface,
                                                            const joint_limits_interface::JointLimits &limits,
                                                            const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;
    virtual hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface,
                                                            const joint_limits_interface::JointLimits &limits,
                                                            const joint_limits_interface::SoftJointLimits *soft_limits = 0) = 0;

    virtual void enforceLimits(const ros::Duration &period, bool reset) = 0;
    virtual void enableLimits(bool enable) = 0;
};

}  // namespace canopen

#endif /* CANOPEN_MOTOR_NODE_HANDLE_LAYER_BASE_H_ */
