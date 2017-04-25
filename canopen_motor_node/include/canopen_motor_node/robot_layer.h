
#ifndef CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_
#define CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_

#include <boost/unordered_map.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/urdfdom_compatibility.h>
#include <urdf/model.h>
#include <canopen_402/base.h>
#include <canopen_motor_node/handle_layer_base.h>


namespace canopen {

class RobotLayer : public LayerGroupNoDiag<HandleLayerBase>, public hardware_interface::RobotHW{
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface pos_interface_;
    hardware_interface::VelocityJointInterface vel_interface_;
    hardware_interface::EffortJointInterface eff_interface_;

    joint_limits_interface::PositionJointSoftLimitsInterface pos_soft_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pos_saturation_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_soft_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vel_saturation_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface eff_soft_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface eff_saturation_interface_;

    ros::NodeHandle nh_;
    urdf::Model urdf_;

    typedef boost::unordered_map< std::string, boost::shared_ptr<HandleLayerBase> > HandleMap;
    HandleMap handles_;
    struct SwitchData {
        boost::shared_ptr<HandleLayerBase> handle;
        canopen::MotorBase::OperationMode mode;
        bool enforce_limits;
    };
    typedef std::vector<SwitchData>  SwitchContainer;
    typedef boost::unordered_map<std::string, SwitchContainer>  SwitchMap;
    SwitchMap switch_map_;

    boost::atomic<bool> first_init_;

    void stopControllers(const std::vector<std::string> controllers);
public:
    void add(const std::string &name, boost::shared_ptr<HandleLayerBase> handle);
    RobotLayer(ros::NodeHandle nh);
    urdf::JointConstSharedPtr getJoint(const std::string &n) const { return urdf_.getJoint(n); }

    virtual void handleInit(canopen::LayerStatus &status);
    void enforce(const ros::Duration &period, bool reset);
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
};

}  // namespace canopen

#endif
