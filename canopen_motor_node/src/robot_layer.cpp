

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/SwitchController.h>

#include <canopen_motor_node/robot_layer.h>

#include "interface_mapping.h"

using namespace canopen;

InterfaceMapping g_interface_mapping;

void RobotLayer::stopControllers(const std::vector<std::string> controllers){
    controller_manager_msgs::SwitchController srv;
    srv.request.stop_controllers = controllers;
    srv.request.strictness = srv.request.BEST_EFFORT;
    boost::thread call(boost::bind(ros::service::call<controller_manager_msgs::SwitchController>, "controller_manager/switch_controller", srv));
    call.detach();
}

void RobotLayer::add(const std::string &name, boost::shared_ptr<HandleLayerBase> handle){
    LayerGroupNoDiag::add(handle);
    handles_.insert(std::make_pair(name, handle));
}

RobotLayer::RobotLayer(ros::NodeHandle nh) : LayerGroupNoDiag<HandleLayerBase>("RobotLayer"), nh_(nh), first_init_(true)
{
    registerInterface(&state_interface_);
    registerInterface(&pos_interface_);
    registerInterface(&vel_interface_);
    registerInterface(&eff_interface_);

    urdf_.initParam("robot_description");
}

void RobotLayer::handleInit(LayerStatus &status){
    if(first_init_){
        for(HandleMap::iterator it = handles_.begin(); it != handles_.end(); ++it){
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            urdf::JointConstSharedPtr joint = getJoint(it->first);

            if(!joint){
                status.error("joint " + it->first + " not found");
                return;
            }

            bool has_joint_limits = joint_limits_interface::getJointLimits(joint, limits);

            has_joint_limits = joint_limits_interface::getJointLimits(it->first, nh_, limits) || has_joint_limits;

            bool has_soft_limits = has_joint_limits && joint_limits_interface::getSoftJointLimits(joint, soft_limits);

            if(!has_joint_limits){
                ROS_WARN_STREAM("No limits found for " << it->first);
            }

            it->second->registerHandle(state_interface_);

            const hardware_interface::JointHandle *h  = 0;

            it->second->registerHandle(pos_interface_, limits, has_soft_limits ? &soft_limits : 0);
            it->second->registerHandle(vel_interface_, limits, has_soft_limits ? &soft_limits : 0);
            it->second->registerHandle(eff_interface_, limits, has_soft_limits ? &soft_limits : 0);
        }
        first_init_ = false;
    }
    LayerGroupNoDiag::handleInit(status);
}

void RobotLayer::enforce(const ros::Duration &period, bool reset){
    for(HandleMap::iterator it = handles_.begin(); it != handles_.end(); ++it){
        it->second->enforceLimits(period, reset);
    }
}
class ModeLookup {
    int default_mode_;
    bool has_default_mode_;
    std::map<std::string, int> lookup_;
    bool has_lookup_;
public:
    ModeLookup(ros::NodeHandle &nh_c){
        has_default_mode_ = nh_c.getParam("required_drive_mode", default_mode_);
        has_lookup_ = nh_c.getParam("required_drive_modes", lookup_);
    }
    bool hasModes() { return has_default_mode_ || has_lookup_; }
    bool hasMixedModes() { return has_lookup_; }
    bool getMode(MotorBase::OperationMode &om, const std::string &key) {
        std::map<std::string, int>::iterator f = lookup_.find(key);
        if(f != lookup_.end()){
            om = MotorBase::OperationMode(f->second);
            return true;
        }else if (has_default_mode_) {
            om = MotorBase::OperationMode(default_mode_);
            return true;
        }
        return false;
    }
};
bool RobotLayer::prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) {
    // compile-time check for mode switching support in ros_control
    (void) &hardware_interface::RobotHW::prepareSwitch; // please upgrade to ros_control/contoller_manager 0.9.4 or newer

    // stop handles
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){

        if(switch_map_.find(controller_it->name) == switch_map_.end()){
            ROS_ERROR_STREAM(controller_it->name << " was not started before");
            return false;
        }
    }

    // start handles
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
        SwitchContainer to_switch;
        ros::NodeHandle nh(nh_, controller_it->name);
        ModeLookup ml(nh);

        std::set<std::string> claimed_interfaces;

        if(controller_it->claimed_resources.size() > 0){
            for (std::vector<hardware_interface::InterfaceResources>::const_iterator cres_it = controller_it->claimed_resources.begin(); cres_it != controller_it->claimed_resources.end(); ++cres_it){
                for (std::set<std::string>::const_iterator res_it = cres_it->resources.begin(); res_it != cres_it->resources.end(); ++res_it){
                    claimed_interfaces.insert(cres_it->hardware_interface);
                    if(!ml.hasModes()){
                        ROS_ERROR_STREAM("Please set required_drive_mode(s) for controller " << controller_it->name);
                        return false;
                    }
                    if(claimed_interfaces.size() > 1 && !ml.hasMixedModes()){
                        ROS_ERROR_STREAM("controller "<< controller_it->name << " has mixed interfaces, please set required_drive_modes.");
                        return false;
                    }

                    boost::unordered_map< std::string, boost::shared_ptr<HandleLayerBase> >::const_iterator h_it = handles_.find(*res_it);

                    const std::string & joint = *res_it;

                    if(h_it == handles_.end()){
                        ROS_ERROR_STREAM(joint << " not found");
                        return false;
                    }
                    SwitchData sd;
                    sd.enforce_limits = nh.param("enforce_limits", true);

                    if(!ml.getMode(sd.mode, joint)){
                        ROS_ERROR_STREAM("could not determine drive mode for " << joint);
                        return false;
                    }

                    if(g_interface_mapping.hasConflict(cres_it->hardware_interface, sd.mode)){
                        ROS_ERROR_STREAM(cres_it->hardware_interface << " cannot be provided in mode " << sd.mode);
                        return false;
                    }

                    HandleLayerBase::CanSwitchResult res = h_it->second->canSwitch(sd.mode);

                    switch(res){
                        case HandleLayerBase::NotSupported:
                            ROS_ERROR_STREAM("Mode " << sd.mode << " is not available for " << joint);
                            return false;
                        case HandleLayerBase::NotReadyToSwitch:
                            ROS_ERROR_STREAM(joint << " is not ready to switch mode");
                            return false;
                        case HandleLayerBase::ReadyToSwitch:
                        case HandleLayerBase::NoNeedToSwitch:
                            sd.handle = h_it->second;
                            to_switch.push_back(sd);
                    }
                }
            }
        }
        switch_map_.insert(std::make_pair(controller_it->name, to_switch));
    }

    // perform mode switches
    boost::unordered_set<boost::shared_ptr<HandleLayerBase> > to_stop;
    std::vector<std::string> failed_controllers;
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){
        SwitchContainer &to_switch = switch_map_.at(controller_it->name);
        for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
            to_stop.insert(it->handle);
        }
    }
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
        SwitchContainer &to_switch = switch_map_.at(controller_it->name);
        bool okay = true;
        for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
            it->handle->switchMode(MotorBase::No_Mode); // stop all
        }
        for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
            if(!it->handle->switchMode(it->mode)){
                failed_controllers.push_back(controller_it->name);
                ROS_ERROR_STREAM("Could not switch one joint for " << controller_it->name << ", will stop all related joints and the controller.");
                for(RobotLayer::SwitchContainer::iterator stop_it = to_switch.begin(); stop_it != to_switch.end(); ++stop_it){
                    to_stop.insert(stop_it->handle);
                }
                okay = false;
                break;
            }else{
                it->handle->enableLimits(it->enforce_limits);
            }
            to_stop.erase(it->handle);
        }
    }
    for(boost::unordered_set<boost::shared_ptr<HandleLayerBase> >::iterator it = to_stop.begin(); it != to_stop.end(); ++it){
        (*it)->switchMode(MotorBase::No_Mode);
    }
    if(!failed_controllers.empty()){
        stopControllers(failed_controllers);
        // will not return false here since this would prevent other controllers to be started and therefore lead to an inconsistent state
    }

    return true;
}

void RobotLayer::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) {
    std::vector<std::string> failed_controllers;
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
        try{
            SwitchContainer &to_switch = switch_map_.at(controller_it->name);
            for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
                if(!it->handle->forwardForMode(it->mode)){
                    failed_controllers.push_back(controller_it->name);
                    ROS_ERROR_STREAM("Could not switch one joint for " << controller_it->name << ", will stop all related joints and the controller.");
                    for(RobotLayer::SwitchContainer::iterator stop_it = to_switch.begin(); stop_it != to_switch.end(); ++stop_it){
                        it->handle->switchMode(MotorBase::No_Mode);
                    }
                    break;
                }
            }

        }catch(const std::out_of_range&){
            ROS_ERROR_STREAM("Conttroller " << controller_it->name << "not found, will stop it");
            failed_controllers.push_back(controller_it->name);
        }
    }
    if(!failed_controllers.empty()){
        stopControllers(failed_controllers);
    }
}
