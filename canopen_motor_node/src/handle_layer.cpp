
#include <canopen_motor_node/handle_layer.h>
#include "interface_mapping.h"

using namespace canopen;


template<typename T > class LimitsHandle {
    T limits_handle_;
public:
    LimitsHandle(const T &handle) : limits_handle_(handle) {}
    virtual void enforce(const ros::Duration &period) { limits_handle_.enforceLimits(period); }
    virtual void reset() {}
};

template<> void LimitsHandle<joint_limits_interface::PositionJointSaturationHandle>::reset() { limits_handle_.reset(); }
template<>void LimitsHandle<joint_limits_interface::PositionJointSoftLimitsHandle>::reset() { limits_handle_.reset(); }

bool HandleLayer::select(const MotorBase::OperationMode &m){
    CommandMap::iterator it = commands_.find(m);
    if(it == commands_.end()) return false;
    jh_ = it->second;
    return true;
}

HandleLayer::HandleLayer(const std::string &name, const boost::shared_ptr<MotorBase> & motor, const boost::shared_ptr<ObjectStorage> storage,  XmlRpc::XmlRpcValue & options)
: HandleLayerBase(name + " Handle"), motor_(motor), variables_(storage), jsh_(name, &pos_, &vel_, &eff_), jph_(jsh_, &cmd_pos_), jvh_(jsh_, &cmd_vel_), jeh_(jsh_, &cmd_eff_), jh_(0), forward_command_(false),
  filter_pos_("double"), filter_vel_("double"), filter_eff_("double"), options_(options), enable_limits_(true)
{
   commands_[MotorBase::No_Mode] = 0;

   std::string p2d("rint(rad2deg(pos)*1000)"), v2d("rint(rad2deg(vel)*1000)"), e2d("rint(eff)");
   std::string p2r("deg2rad(obj6064)/1000"), v2r("deg2rad(obj606C)/1000"), e2r("0");

   if(options.hasMember("pos_unit_factor") || options.hasMember("vel_unit_factor") || options.hasMember("eff_unit_factor")){
       const std::string reason("*_unit_factor parameters are not supported anymore, please migrate to conversion functions.");
       ROS_FATAL_STREAM(reason);
       throw std::invalid_argument(reason);
   }

   if(options.hasMember("pos_to_device")) p2d = (const std::string&) options["pos_to_device"];
   if(options.hasMember("pos_from_device")) p2r = (const std::string&) options["pos_from_device"];

   if(options.hasMember("vel_to_device")) v2d = (const std::string&) options["vel_to_device"];
   if(options.hasMember("vel_from_device")) v2r = (const std::string&) options["vel_from_device"];

   if(options.hasMember("eff_to_device")) e2d = (const std::string&) options["eff_to_device"];
   if(options.hasMember("eff_from_device")) e2r = (const std::string&) options["eff_from_device"];

   conv_target_pos_.reset(new UnitConverter(p2d, boost::bind(assignVariable, "pos", &cmd_pos_, _1)));
   conv_target_vel_.reset(new UnitConverter(v2d, boost::bind(assignVariable, "vel", &cmd_vel_, _1)));
   conv_target_eff_.reset(new UnitConverter(e2d, boost::bind(assignVariable, "eff", &cmd_eff_, _1)));

   conv_pos_.reset(new UnitConverter(p2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
   conv_vel_.reset(new UnitConverter(v2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
   conv_eff_.reset(new UnitConverter(e2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
}

HandleLayer::CanSwitchResult HandleLayer::canSwitch(const MotorBase::OperationMode &m){
    if(!motor_->isModeSupported(m) || commands_.find(m) == commands_.end()){
        return NotSupported;
    }else if(motor_->getMode() == m){
        return NoNeedToSwitch;
    }else if(motor_->getLayerState() == Ready){
        return ReadyToSwitch;
    }else{
        return NotReadyToSwitch;
    }
}

bool HandleLayer::switchMode(const MotorBase::OperationMode &m){
    if(motor_->getMode() != m){
        forward_command_ = false;
        jh_ = 0; // disconnect handle
        if(!motor_->enterModeAndWait(m)){
            ROS_ERROR_STREAM(jsh_.getName() << "could not enter mode " << (int)m);
            LayerStatus s;
            motor_->halt(s);
            return false;
        }
    }
    return select(m);
}

bool HandleLayer::forwardForMode(const MotorBase::OperationMode &m){
    if(motor_->getMode() == m){
        forward_command_ = true;
        return true;
    }
    return false;
}


template<typename T> void addLimitsHandle(std::vector<LimitsHandleBase::Ptr> &limits, const T &t) {
    limits.push_back(LimitsHandleBase::Ptr( (LimitsHandleBase *) new LimitsHandle<T> (t) ));
}

hardware_interface::JointHandle* HandleLayer::registerHandle(hardware_interface::PositionJointInterface &iface,
                                                             const joint_limits_interface::JointLimits &limits,
                                                             const joint_limits_interface::SoftJointLimits *soft_limits){
    hardware_interface::JointHandle* h = addHandle(iface, &jph_, g_interface_mapping.getInterfaceModes("hardware_interface::PositionJointInterface"));
    if(h &&  limits.has_position_limits){
        addLimitsHandle(limits_, joint_limits_interface::PositionJointSaturationHandle(*h, limits));
        if(soft_limits){
            addLimitsHandle(limits_, joint_limits_interface::PositionJointSoftLimitsHandle(*h, limits, *soft_limits));
        }
    }
    return h;
}

hardware_interface::JointHandle* HandleLayer::registerHandle(hardware_interface::VelocityJointInterface &iface,
                                                             const joint_limits_interface::JointLimits&limits,
                                                             const joint_limits_interface::SoftJointLimits *soft_limits){
    hardware_interface::JointHandle* h = addHandle(iface, &jvh_, g_interface_mapping.getInterfaceModes("hardware_interface::VelocityJointInterface"));
    if(h &&  limits.has_velocity_limits){
        addLimitsHandle(limits_, joint_limits_interface::VelocityJointSaturationHandle(*h, limits));
        if(soft_limits){
            addLimitsHandle(limits_, joint_limits_interface::VelocityJointSoftLimitsHandle(*h, limits, *soft_limits));
        }
    }
    return h;
}

hardware_interface::JointHandle* HandleLayer::registerHandle(hardware_interface::EffortJointInterface &iface,
                                                             const joint_limits_interface::JointLimits&limits,
                                                             const joint_limits_interface::SoftJointLimits *soft_limits){
    hardware_interface::JointHandle* h = addHandle(iface, &jeh_, g_interface_mapping.getInterfaceModes("hardware_interface::EffortJointInterface"));
    if(h &&  limits.has_effort_limits){
        addLimitsHandle(limits_, joint_limits_interface::EffortJointSaturationHandle(*h, limits));
        if(soft_limits){
            addLimitsHandle(limits_, joint_limits_interface::EffortJointSoftLimitsHandle(*h, limits, *soft_limits));
        }
    }
    return h;
}

void HandleLayer::handleRead(LayerStatus &status, const LayerState &current_state) {
    if(current_state > Shutdown){
        variables_.sync();
        filter_pos_.update(conv_pos_->evaluate(), pos_);
        filter_vel_.update(conv_vel_->evaluate(), vel_);
        filter_eff_.update(conv_eff_->evaluate(), eff_);
    }
}
void HandleLayer::handleWrite(LayerStatus &status, const LayerState &current_state) {
    if(current_state == Ready){
        hardware_interface::JointHandle* jh = 0;
        if(forward_command_) jh = jh_;

        if(jh == &jph_){
            motor_->setTarget(conv_target_pos_->evaluate());
            cmd_vel_ = vel_;
            cmd_eff_ = eff_;
        }else if(jh == &jvh_){
            motor_->setTarget(conv_target_vel_->evaluate());
            cmd_pos_ = pos_;
            cmd_eff_ = eff_;
        }else if(jh == &jeh_){
            motor_->setTarget(conv_target_eff_->evaluate());
            cmd_pos_ = pos_;
            cmd_vel_ = vel_;
        }else{
            cmd_pos_ = pos_;
            cmd_vel_ = vel_;
            cmd_eff_ = eff_;
            if(jh) status.warn("unsupported mode active");
        }
    }
}

bool prepareFilter(const std::string& joint_name, const std::string& filter_name,  filters::FilterChain<double> &filter, XmlRpc::XmlRpcValue & options, canopen::LayerStatus &status){
    filter.clear();
    if(options.hasMember(filter_name)){
        if(!filter.configure(options[filter_name],joint_name + "/" + filter_name)){
            status.error("could not configure " + filter_name+ " for " + joint_name);
            return false;
        }
    }

    return true;
}

bool HandleLayer::prepareFilters(canopen::LayerStatus &status){
    return prepareFilter(jsh_.getName(), "position_filters", filter_pos_, options_, status) &&
       prepareFilter(jsh_.getName(), "velocity_filters", filter_vel_, options_, status) &&
       prepareFilter(jsh_.getName(), "effort_filters", filter_eff_, options_, status);
}

void HandleLayer::handleInit(LayerStatus &status){
    // TODO: implement proper init
    conv_pos_->reset();
    conv_vel_->reset();
    conv_eff_->reset();
    conv_target_pos_->reset();
    conv_target_vel_->reset();
    conv_target_eff_->reset();


    if(prepareFilters(status))
    {
        handleRead(status, Layer::Ready);
    }
}

void HandleLayer::enforceLimits(const ros::Duration &period, bool reset){
    for(std::vector<LimitsHandleBase::Ptr>::iterator it = limits_.begin(); it != limits_.end(); ++it){
        if(reset) (*it)->reset();
        if(enable_limits_) (*it)->enforce(period);
    }
}

void HandleLayer::enableLimits(bool enable){
    enable_limits_ = enable;
}
