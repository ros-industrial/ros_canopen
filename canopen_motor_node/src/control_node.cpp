#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/chain_ros.h>

#include <canopen_402/canopen_402.h>
#include <canopen_402/enums_402.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>

#include <controller_manager/controller_manager.h>

using namespace can;
using namespace canopen;


//typedef Node_402 MotorNode;

class MotorNode : public Node_402{
    double pos_unit_factor;
    double vel_unit_factor;
    double eff_unit_factor;
public:
   MotorNode(boost::shared_ptr <canopen::Node> n, const std::string &name, XmlRpc::XmlRpcValue & options) : Node_402(n, name), pos_unit_factor(360*1000/(2*M_PI)),vel_unit_factor(360*1000/(2*M_PI)), eff_unit_factor(1) {
       if(options.hasMember("pos_unit_factor")) pos_unit_factor = options["pos_unit_factor"];
       if(options.hasMember("vel_unit_factor")) vel_unit_factor = options["vel_unit_factor"];
       if(options.hasMember("eff_unit_factor")) eff_unit_factor = options["eff_unit_factor"];
   }
   const double getActualPos() { return Node_402::getActualPos() / pos_unit_factor; }
   const double getActualVel() { return Node_402::getActualVel() / vel_unit_factor; }
   const double getActualEff() { return Node_402::getActualEff() / eff_unit_factor; }
   
   void setTargetPos(const double &v) { Node_402::setTargetPos(v*pos_unit_factor); }
   void setTargetVel(const double &v) { Node_402::setTargetVel(v*vel_unit_factor); }
   void setTargetEff(const double &v) { Node_402::setTargetEff(v*eff_unit_factor); }
   
   const double getTargetPos() { return Node_402::getTargetPos() / pos_unit_factor; }
   const double getTargetVel() { return Node_402::getTargetVel() / vel_unit_factor; }
   const double getTargetEff() { return Node_402::getTargetEff() / eff_unit_factor; }
};

class HandleLayer: public Layer{
    boost::shared_ptr<MotorNode> motor_;
    double pos_, vel_, eff_;
    double cmd_pos_, cmd_vel_, cmd_eff_;

    
    hardware_interface::JointStateHandle jsh_;
    hardware_interface::JointHandle jph_, jvh_, jeh_, *jh_;

    typedef boost::unordered_map< OperationMode,hardware_interface::JointHandle* > CommandMap;
    CommandMap commands_;

    template <typename T> hardware_interface::JointHandle* addHandle( T &iface, hardware_interface::JointHandle *jh,  const std::vector<OperationMode> & modes){

        uint32_t mode_mask = 0;
        for(size_t i=0; i < modes.size(); ++i){
            if(motor_->isModeSupported(modes[i]))
                mode_mask |= MotorNode::getModeMask(modes[i]);
        }
        if(mode_mask == 0) return 0;

        iface.registerHandle(*jh);

        for(size_t i=0; i < modes.size(); ++i){
            commands_[modes[i]] = jh;
        }
        return jh;
    }
    bool select(const OperationMode &m){
        CommandMap::iterator it = commands_.find(m);
        if(it == commands_.end()) return false;
        jh_ = it->second;
        return true;
    }
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<MotorNode> & motor)
    : Layer(name + " Handle"), motor_(motor), jsh_(name, &pos_, &vel_, &eff_), jph_(jsh_, &cmd_pos_), jvh_(jsh_, &cmd_vel_), jeh_(jsh_, &cmd_eff_), jh_(0) {}

    int canSwitch(const OperationMode &m){
       if(motor_->getMode() == m) return -1;
       if(commands_.find(m) != commands_.end()) return 1;
       return 0;
    }
    bool switchMode(const OperationMode &m){
        CommandMap::iterator it = commands_.find(m);
        if(it == commands_.end()) return false;

        return motor_->enterModeAndWait(m) && select(m);
    }

    void registerHandle(hardware_interface::JointStateInterface &iface){
        iface.registerHandle(jsh_);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface){
        std::vector<OperationMode> modes;
        modes.push_back(Profiled_Position);
        modes.push_back(Interpolated_Position);
        modes.push_back(Cyclic_Synchronous_Position);
        return addHandle(iface, &jph_, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface){
        std::vector<OperationMode> modes;
        modes.push_back(Velocity);
        modes.push_back(Profiled_Velocity);
        modes.push_back(Cyclic_Synchronous_Velocity);
        return addHandle(iface, &jvh_, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface){
        std::vector<OperationMode> modes;
        modes.push_back(Profiled_Torque);
        modes.push_back(Cyclic_Synchronous_Torque);
        return addHandle(iface, &jeh_, modes);
    }    
private:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            cmd_pos_ = pos_ = motor_->getActualPos();
            cmd_vel_ = vel_ = motor_->getActualVel();
            cmd_eff_ = eff_ = motor_->getActualEff();
            if(!jh_){
                OperationMode m = motor_->getMode();
                if(m != No_Mode && !select(m)){
                    status.error("No mode selected");
                }
            }
            if(jh_ == &jph_){
                cmd_pos_ = motor_->getTargetPos();
            }else if(jh_ == &jvh_){
                cmd_vel_ = motor_->getTargetVel();
            }else if(jh_ == &jeh_){
                cmd_eff_ = motor_->getTargetEff();
            }
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state == Ready){
            if(jh_){
                if(jh_ == &jph_){
                    motor_->setTargetPos(cmd_pos_);
                }else if(jh_ == &jvh_){
                    motor_->setTargetVel(cmd_vel_);
                }else if(jh_ == &jeh_){
                    motor_->setTargetEff(cmd_eff_);
                }
            }else if (motor_->getMode() != No_Mode){
                status.warn("unsupported mode active");
            }
        }
    }
    virtual void handleInit(LayerStatus &status){
        // TODO: implement proper init
        handleRead(status, Layer::Ready);
    }
    
    virtual void handleDiag(LayerReport &report) { /* nothing to do */ }
    virtual void handleShutdown(LayerStatus &status) { /* nothing to do */ }
    virtual void handleHalt(LayerStatus &status) { /* TODO */ }
    virtual void handleRecover(LayerStatus &status) { LOG("RECOVER"); handleRead(status, Layer::Ready); }
    
};


class RobotLayer : public LayerGroupNoDiag<HandleLayer>, public hardware_interface::RobotHW{
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

    typedef boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> > HandleMap;
    HandleMap handles_;
public:
    typedef std::vector<std::pair <boost::shared_ptr<HandleLayer>, OperationMode> >  SwitchContainer;

    virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &info_list, SwitchContainer &to_switch) {
        to_switch.reserve(handles_.size());

        for (std::list<hardware_interface::ControllerInfo>::const_iterator info_it = info_list.begin(); info_it != info_list.end(); ++info_it){
            ros::NodeHandle nh(nh_,info_it->name);
            int mode;
            if(!nh.getParam("required_drive_mode", mode)) continue;

            for (std::set<std::string>::const_iterator res_it = info_it->resources.begin(); res_it != info_it->resources.end(); ++res_it){
                boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> >::const_iterator h_it = handles_.find(*res_it);

                if(h_it == handles_.end()){
                    ROS_ERROR_STREAM(*res_it << " not found");
                    return false;
                }
                if(int res = h_it->second->canSwitch((OperationMode)mode)){
                    if(res > 0) to_switch.push_back(std::make_pair(h_it->second, OperationMode(mode)));
                }else{
                    ROS_ERROR_STREAM("Mode " << mode << " is not available for " << *res_it);
                    return false;
                }
            }
        }
        return true;
    }

    void add(const std::string &name, boost::shared_ptr<HandleLayer> handle){
        LayerGroupNoDiag::add(handle);
        handles_.insert(std::make_pair(name, handle));
    }
    RobotLayer(ros::NodeHandle nh) : LayerGroupNoDiag<HandleLayer>("RobotLayer"), nh_(nh)
    {
        registerInterface(&state_interface_);
        registerInterface(&pos_interface_);
        registerInterface(&vel_interface_);
        registerInterface(&eff_interface_);


        registerInterface(&pos_saturation_interface_);
        registerInterface(&pos_soft_limits_interface_);
        registerInterface(&vel_saturation_interface_);
        registerInterface(&vel_soft_limits_interface_);
        registerInterface(&eff_saturation_interface_);
        registerInterface(&eff_soft_limits_interface_);

        urdf_.initParam("robot_description");
    }

    boost::shared_ptr<const urdf::Joint> getJoint(const std::string &n) const { return urdf_.getJoint(n); }

    virtual void handleInit(LayerStatus &status){
        urdf::Model urdf;
        urdf.initParam("robot_description");

        for(HandleMap::iterator it = handles_.begin(); it != handles_.end(); ++it){
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            boost::shared_ptr<const urdf::Joint> joint = getJoint(it->first);
            
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

            h = it->second->registerHandle(pos_interface_);
            if(h && has_joint_limits){
                joint_limits_interface::PositionJointSaturationHandle sathandle(*h, limits);
                pos_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::PositionJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    pos_soft_limits_interface_.registerHandle(softhandle);
                }
            }
            h = it->second->registerHandle(vel_interface_);
            if(h && has_joint_limits){
                joint_limits_interface::VelocityJointSaturationHandle sathandle(*h, limits);
                vel_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::VelocityJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    vel_soft_limits_interface_.registerHandle(softhandle);
                }
            }
            h = it->second->registerHandle(eff_interface_);
            if(h && has_joint_limits){
                joint_limits_interface::EffortJointSaturationHandle sathandle(*h, limits);
                eff_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::EffortJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    eff_soft_limits_interface_.registerHandle(softhandle);
                }
            }

        }
        LayerGroupNoDiag::handleInit(status);
    }

    void enforce(const ros::Duration &period, bool reset){
        if(reset){
            pos_saturation_interface_.reset();
            pos_soft_limits_interface_.reset();
        }
        pos_saturation_interface_.enforceLimits(period);
        pos_soft_limits_interface_.enforceLimits(period);
        vel_saturation_interface_.enforceLimits(period);
        vel_soft_limits_interface_.enforceLimits(period);
        eff_saturation_interface_.enforceLimits(period);
        eff_soft_limits_interface_.enforceLimits(period);
    }

};

 class ControllerManager : public controller_manager::ControllerManager{
    boost::shared_ptr<RobotLayer> robot_;

    bool recover_;
    ros::Time last_time_;
    boost::mutex mutex_;
 public:
    virtual bool notifyHardwareInterface(const std::list<hardware_interface::ControllerInfo> &info_list) {
        RobotLayer::SwitchContainer to_switch;

        if(!robot_->canSwitch(info_list, to_switch)) return false;

        if(!to_switch.empty()){
            boost::mutex::scoped_lock lock(mutex_);
            for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
                if(!it->first->switchMode(it->second)) return false;
            }
            recover_ = true;
        }

        controller_manager::ControllerManager::notifyHardwareInterface(info_list); //compile-time check for ros_control notifyHardwareInterface support
        return true;

    }

    void update(){
        boost::mutex::scoped_lock lock(mutex_, boost::try_to_lock);
        if(!lock) return;

        ros::Time now = ros::Time::now();
        ros::Duration period(now -last_time_);

        last_time_ = now;
        controller_manager::ControllerManager::update(now, period, recover_);
        // robot_->enforce(period, recover_);
        recover_ = false;
   }

    void recover() { boost::mutex::scoped_lock lock(mutex_); recover_ = true; }

    ControllerManager(boost::shared_ptr<RobotLayer> robot, ros::NodeHandle nh)  : controller_manager::ControllerManager(robot.get(), nh), robot_(robot), recover_(false), last_time_(ros::Time::now()) {}
 };

class ControllerManagerLayer : public Layer {
    boost::shared_ptr<ControllerManager> cm_;
    boost::shared_ptr<RobotLayer> robot_;
    ros::NodeHandle nh_;

public:
    ControllerManagerLayer(const boost::shared_ptr<RobotLayer> robot, const ros::NodeHandle &nh)
    :Layer("ControllerManager"), robot_(robot), nh_(nh) {
    }

    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            if(!cm_) status.error("controller_manager is not intialized");
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Init){
            if(!cm_) status.error("controller_manager is not intialized");
            else cm_->update();
        }
    }
    virtual void handleDiag(LayerReport &report) { /* nothing to do */ }
    virtual void handleHalt(LayerStatus &status) { /* nothing to do (?) */ }

    virtual void handleInit(LayerStatus &status) {
        if(cm_){
            status.warn("controller_manager is already intialized");
        }else{
            cm_.reset(new ControllerManager(robot_, nh_));
        }
        cm_->recover();
    }
    virtual void handleRecover(LayerStatus &status) {
        if(!cm_) status.error("controller_manager is not intialized");
        else cm_->recover();
    }
    virtual void handleShutdown(LayerStatus &status) {
        cm_.reset();
    }
};

template<typename MotorNodeType> class MotorChain : public RosChain{
    boost::shared_ptr< LayerGroupNoDiag<MotorNodeType> > motors_;
    boost::shared_ptr<RobotLayer> robot_layer_;

    boost::shared_ptr< ControllerManagerLayer> cm_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger)
    {
        std::string name = params["name"];
        std::string &joint = name;
        if(params.hasMember("joint")) joint.assign(params["joint"]);

        if(!robot_layer_->getJoint(joint)){
            ROS_ERROR_STREAM("joint " + joint + " was not found in URDF");
            return false;
        }

        boost::shared_ptr<MotorNode> motor( new MotorNode(node, name + "_motor", params));
        motors_->add(motor);
        logger->add(motor);

        boost::shared_ptr<HandleLayer> handle( new HandleLayer(joint, motor));
        robot_layer_->add(joint, handle);
        logger->add(handle);

        return true;
    }

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv){}

    virtual bool setup() {
        motors_.reset( new LayerGroupNoDiag<MotorNode>("402 Layer"));
        robot_layer_.reset( new RobotLayer(nh_));
        cm_.reset(new ControllerManagerLayer(robot_layer_, nh_));

        if(RosChain::setup()){
            add(motors_);
            add(robot_layer_);

            add(cm_);

            return true;
        }

        return false;
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "canopen_chain_node_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  MotorChain<MotorNode> chain(nh, nh_priv);

  if(!chain.setup()){
      return -1;
  }

  ros::waitForShutdown();
  return 0;
}
