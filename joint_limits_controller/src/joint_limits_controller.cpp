#include <joint_limits_controller/limited_joint_handle.h>
#include <urdf/model.h>
#include <ros/ros.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_loader.h>

namespace joint_limits_controller {

class JointLimitsController : public controller_interface::ControllerBase {
    pluginlib::ClassLoader<controller_interface::ControllerBase> loader_;
    boost::shared_ptr<controller_interface::ControllerBase> controller_;
    std::vector<boost::shared_ptr<LimitedJointHandle> > limit_handles_;

    template<typename T> bool track(const hardware_interface::JointHandle &handle, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh) {
        std::string name = handle.getName();
        LimitedJointHandle::Limits limits(urdf_.getJoint(name));
        limits.apply(name, root_nh, true);
        limits.apply(name, controller_nh, true);
        boost::shared_ptr<LimitedJointHandle> ptr(new T(handle, limits));
        limit_handles_.push_back(ptr);
        return true; // TODO: Might fail fo various reasons, add proper checks
    }

    template<typename T> bool track(hardware_interface::JointCommandInterface *interface, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh, std::set<std::string>& claimed_resources){
        if(!interface) {
            ROS_ERROR("Interface invalid");
            return false;
        }
        for(std::set<std::string>::iterator it = claimed_resources.begin(); it != claimed_resources.end(); ++it){
            if(!track<T>(interface->getHandle(*it), root_nh, controller_nh)) return false;
        }
        return true;
    }

    urdf::Model urdf_;
public:
    JointLimitsController()
    : loader_("controller_interface", "controller_interface::ControllerBase")
    {}

    virtual std::string getHardwareInterfaceType() const { return controller_ ? controller_->getHardwareInterfaceType() : std::string(); }

    virtual bool initRequest(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh, std::set<std::string>& claimed_resources){

        if (state_ != CONSTRUCTED){
            ROS_ERROR("Controller is not in CONSTRUCTED state.");
            return false;
        }

        std::string controller_type;
        if(!controller_nh.getParam("type", controller_type)){
            ROS_ERROR("Missing controller param");
            return false;
        }
        if(controller_type == "joint_limits_controller::JointLimitsController"){
            if(!controller_nh.getParam("controller_type", controller_type)){
                ROS_ERROR("Missing controller param");
                return false;
            }
        }

        if(!loader_.isClassAvailable(controller_type)){
            ROS_ERROR_STREAM("Controller type " << controller_type << " is not available");
            return false;
        }

        std::string has_robot_description_fqn;

        bool has_robot_description = true;

        if(controller_nh.searchParam("has_robot_description", has_robot_description_fqn)){
            ros::param::get(has_robot_description_fqn, has_robot_description);
        }
        if(has_robot_description){
            urdf_.initParam("robot_description");
        }

        controller_ = loader_.createInstance(controller_type);

        if(!controller_){
            ROS_ERROR_STREAM("Failed to load controller of type " << controller_type);
            return false;
        }

        if(!controller_->initRequest(hw, root_nh, controller_nh, claimed_resources)) return false;

        bool ok = false;;
        if(getHardwareInterfaceType() == "hardware_interface::PositionJointInterface"){
            ok = track<LimitedPositionJointHandle>(hw->get<hardware_interface::PositionJointInterface>(), root_nh, controller_nh, claimed_resources);
        }else if(getHardwareInterfaceType() == "hardware_interface::VelocityJointInterface"){
            ok = track<LimitedVelocityJointHandle>(hw->get<hardware_interface::VelocityJointInterface>(), root_nh, controller_nh, claimed_resources);
        }else if(getHardwareInterfaceType() == "hardware_interface::EffortJointInterface"){
            ok = track<LimitedEffortJointHandle>(hw->get<hardware_interface::EffortJointInterface>(), root_nh, controller_nh, claimed_resources);
        }else {
            ROS_ERROR_STREAM("Hardware interface " << getHardwareInterfaceType() << " is not supported");

        }

        if(ok) state_ = INITIALIZED;
        return ok;
    }

    virtual void starting(const ros::Time& time){
        if(!controller_) return;
        controller_->starting(time);
        for(std::vector<boost::shared_ptr<LimitedJointHandle> >::iterator it = limit_handles_.begin(); it != limit_handles_.end(); ++it){
            (*it)->recover();
        }
    }
    virtual void update(const ros::Time& time, const ros::Duration& period){
        if(!controller_) return;
        controller_->update(time, period);
        for(std::vector<boost::shared_ptr<LimitedJointHandle> >::iterator it = limit_handles_.begin(); it != limit_handles_.end(); ++it){
            (*it)->enforceLimits(period);
        }
    }

};

} // joint_limits_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( joint_limits_controller::JointLimitsController, controller_interface::ControllerBase);
