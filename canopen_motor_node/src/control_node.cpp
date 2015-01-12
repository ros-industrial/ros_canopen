#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/chain_ros.h>

#include <canopen_402/canopen_402.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>

#include <controller_manager/controller_manager.h>

//// Dummy class
//class MotorNode : public canopen::SimpleLayer{
//    boost::shared_ptr <canopen::Node> n_;
//    volatile bool running;
//    canopen::ObjectStorage::Entry<int32_t> actual_pos;

//public:
//    MotorNode(boost::shared_ptr <canopen::Node> n) : SimpleLayer("MotorNode"), n_(n) {
//        n->getStorage()->entry(actual_pos, 0x6064);
//    }
//    virtual bool read() { return true; }
//    virtual bool write() { return true; }
//    virtual bool report() { return true; }
//    virtual bool init() { running = true; return true; }
//    virtual bool recover()  { return true; }
//    virtual bool shutdown() { running = false; return true; }
//    const double getActualPos() { return running?actual_pos.get():0; }
//    const double getActualVel() { return 0.0; }
//    const double getActualEff() { return 0.0; }
//    void setTargetPos(const double &v) {}
//    void setTargetVel(const double &v) {}
//    void setTargetEff(const double &v) {}
//    const double getTargetPos() { return 0.0; }
//    const double getTargetVel() { return 0.0; }
//    const double getTargetEff() { return 0.0; }
//};

using namespace can;
using namespace canopen;


//typedef Node_402 MotorNode;

class MotorNode : public Node_402{
    double scale_factor;
public:
   MotorNode(boost::shared_ptr <canopen::Node> n, const std::string &name) : Node_402(n, name), scale_factor(360*1000/(2*M_PI)) {
   }
   const double getActualPos() { return Node_402::getActualPos() / scale_factor; }
   const double getActualVel() { return Node_402::getActualVel() / scale_factor; }
   const double getActualEff() { return Node_402::getActualEff() / scale_factor; }
   void setTargetPos(const double &v) { Node_402::setTargetPos(v*scale_factor); }
   void setTargetVel(const double &v) { Node_402::setTargetVel(v*scale_factor); }
   void setTargetEff(const double &v) { Node_402::setTargetEff(v*scale_factor); }
   const double getTargetPos() { return Node_402::getTargetPos() / scale_factor; }
   const double getTargetVel() { return Node_402::getTargetVel() / scale_factor; }
   const double getTargetEff() { return Node_402::getTargetEff() / scale_factor; }
};



template <typename T> class JointHandleWriter : public hardware_interface::JointHandle {
    double value;
    T & obj_;
    void (T::*writer_)(const double &);
    const double (T::*reader_)(void);
public:
    const uint32_t mode_mask_;
    JointHandleWriter(const hardware_interface::JointStateHandle &jsh, T&  obj, void (T::*writer)(const double &), const double (T::*reader)(void), const uint32_t mode_mask)
    : JointHandle(jsh, &value), obj_(obj), writer_(writer), reader_(reader), mode_mask_(mode_mask) { read(); }
    void write() { (obj_.*writer_)(value); }
    void read() { value = (obj_.*reader_)(); }
};

class HandleLayer: public SimpleLayer{
    boost::shared_ptr<MotorNode> motor_;
    double pos, vel, eff;
    hardware_interface::JointStateHandle jsh;
    typedef JointHandleWriter<MotorNode> CommandWriter;
    typedef boost::unordered_map< MotorNode::OperationMode, boost::shared_ptr<CommandWriter> > CommandMap;
    CommandMap commands_;

    template <typename T> hardware_interface::JointHandle* addHandle( T &iface, void (MotorNode::*writer)(const double &), const double (MotorNode::*reader)(void), const std::vector<MotorNode::OperationMode> & modes){
        uint32_t mode_mask = 0;
        for(size_t i=0; i < modes.size(); ++i){
            if(motor_->isModeSupported(modes[i]))
                mode_mask |= MotorNode::getModeMask(modes[i]);
        }
        if(mode_mask == 0) return 0;

        boost::shared_ptr<CommandWriter> jhw (new CommandWriter(jsh, *motor_, writer, reader, mode_mask));

        iface.registerHandle(*jhw);
        for(size_t i=0; i < modes.size(); ++i){
            commands_[modes[i]] = jhw;
        }
        return jhw.get();
    }
    boost::shared_ptr<CommandWriter> jhw_;
    bool select(const MotorNode::OperationMode &m){
        CommandMap::iterator it = commands_.find(m);
        if(it == commands_.end()) return false;

        jhw_ = it->second;
        jhw_->read();
        return true;
    }
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<MotorNode> & motor)
    : SimpleLayer(name + " Handle"), motor_(motor), jsh(name, &pos, &vel, &eff) {}

    int canSwitch(const MotorNode::OperationMode &m){
       if(motor_->getMode() == m) return -1;
       if(commands_.find(m) != commands_.end()) return 1;
       return 0;
    }
    bool switchMode(const MotorNode::OperationMode &m){
        CommandMap::iterator it = commands_.find(m);
        if(it == commands_.end()) return false;

        return motor_->enterModeAndWait(m) && select(m);
    }

    void registerHandle(hardware_interface::JointStateInterface &iface){
        iface.registerHandle(jsh);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface){
        std::vector<MotorNode::OperationMode> modes;
        modes.push_back(MotorNode::Profiled_Position);
        modes.push_back(MotorNode::Interpolated_Position);
        modes.push_back(MotorNode::Cyclic_Synchronous_Position);
        return addHandle(iface, &MotorNode::setTargetPos, &MotorNode::getTargetPos, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface){
        std::vector<MotorNode::OperationMode> modes;
        modes.push_back(MotorNode::Velocity);
        modes.push_back(MotorNode::Profiled_Velocity);
        modes.push_back(MotorNode::Cyclic_Synchronous_Velocity);
        return addHandle(iface,&MotorNode::setTargetVel, &MotorNode::getTargetVel, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface){
        std::vector<MotorNode::OperationMode> modes;
        modes.push_back(MotorNode::Profiled_Torque);
        modes.push_back(MotorNode::Cyclic_Synchronous_Torque);
        return addHandle(iface,&MotorNode::setTargetEff, &MotorNode::getTargetEff, modes);
    }
    virtual bool read() {
        bool okay = true;
        // okay = motor.okay();
        if(okay){
            pos = motor_->getActualPos();
            vel = motor_->getActualVel();
            eff = motor_->getActualEff();
            if(!jhw_){
                MotorNode::OperationMode m = motor_->getMode();
                if(m != MotorNode::No_Mode) return select(m);
            }
            if(jhw_){
                jhw_->read();
            }
        }
        return okay;
    }
    virtual bool write() {
        if(jhw_){
            jhw_->write();
            return true;
        }
        return motor_->getMode() == MotorNode::No_Mode;
    }
    virtual bool report() { return true; }
    virtual bool init() {
        CommandMap::iterator it = commands_.find(motor_->getMode());
        if(it != commands_.end()) jhw_ = it->second;

        return true;
    }
    virtual bool recover() {
        return true;
    }
    virtual bool shutdown(){
        return true;
    }
};

class ControllerManagerLayer : public SimpleLayer, public hardware_interface::RobotHW {
    ros::NodeHandle nh_;
    boost::shared_ptr<controller_manager::ControllerManager> cm_;
    boost::mutex mutex_;
    bool recover_;
    bool paused_;
    ros::Time last_time_;
    ControllerManagerLayer * this_non_const;
    
    void update(){
        ros::Time now = ros::Time::now();
        ros::Duration period(now -last_time_);
        cm_->update(now, period, recover_);
        recover_ = false;
        last_time_ = now;

        pos_saturation_interface_.enforceLimits(period);
        pos_soft_limits_interface_.enforceLimits(period);
        vel_saturation_interface_.enforceLimits(period);
        vel_soft_limits_interface_.enforceLimits(period);
        eff_saturation_interface_.enforceLimits(period);
        eff_soft_limits_interface_.enforceLimits(period);
    }

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
    
    typedef boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> > HandleMap;
    HandleMap handles_;

    void pause(){
        boost::mutex::scoped_lock lock(mutex_);
        if(cm_) paused_ = true;
    }
    void resume(){
        boost::mutex::scoped_lock lock(mutex_);
        if(paused_){
            paused_ = false;
            recover_ = true;
        }
    }
    
public:
    virtual bool checkForConflict(const std::list<hardware_interface::ControllerInfo>& info) const{
        bool in_conflict = RobotHW::checkForConflict(info);
        if(in_conflict) return true;

        typedef std::vector<std::pair <boost::shared_ptr<HandleLayer>, MotorNode::OperationMode> >  SwitchContainer;
        SwitchContainer to_switch;
        to_switch.reserve(handles_.size());

        for (std::list<hardware_interface::ControllerInfo>::const_iterator info_it = info.begin(); info_it != info.end(); ++info_it){
            ros::NodeHandle nh(nh_,info_it->name);
            int mode;
            if(!nh.getParam("required_drive_mode", mode)) continue;

            for (std::set<std::string>::const_iterator res_it = info_it->resources.begin(); res_it != info_it->resources.end(); ++res_it){
                boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> >::const_iterator h_it = handles_.find(*res_it);

                if(h_it == handles_.end()){
                    ROS_ERROR_STREAM(*res_it << " not found");
                    return true;
                }
                if(int res = h_it->second->canSwitch((MotorNode::OperationMode)mode)){
                    if(res > 0) to_switch.push_back(std::make_pair(h_it->second, MotorNode::OperationMode(mode)));
                }else{
                    ROS_ERROR_STREAM("Mode " << mode << " is not available for " << *res_it);
                    return true;
                }
            }
        }

        if(!to_switch.empty()){
            this_non_const->pause();
            for(SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
                // TODO: rollback
                if(!it->first->switchMode(it->second)) return true;
            }
            
            ///call enforceLimits with large period in order to reset their internal prev_cmd_ value!
            ros::Duration period(1000000000.0);
            this_non_const->pos_saturation_interface_.enforceLimits(period);
            this_non_const->pos_soft_limits_interface_.enforceLimits(period);
            this_non_const->vel_saturation_interface_.enforceLimits(period);
            this_non_const->vel_soft_limits_interface_.enforceLimits(period);
            this_non_const->eff_saturation_interface_.enforceLimits(period);
            this_non_const->eff_soft_limits_interface_.enforceLimits(period);

            /*try{  ej_sat_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            try{  ej_limits_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            try{  pj_sat_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            try{  pj_limits_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            try{  vj_sat_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            try{  vj_limits_interface_.enforceLimits(period);  }
            catch(const joint_limits_interface::JointLimitsInterfaceException&){}
            */

            this_non_const->resume();
        }

        return false;
    }

    ControllerManagerLayer(const ros::NodeHandle &nh)
    :SimpleLayer("ControllerManager"), nh_(nh), recover_(false), paused_(false), last_time_(ros::Time::now()), this_non_const(this) {
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
        
    }

    virtual bool read() {
        boost::mutex::scoped_lock lock(mutex_);
        return cm_;
    }
    virtual bool write()  {
        boost::mutex::scoped_lock lock(mutex_);
        if(cm_ && !paused_){
            update();
        }
        return cm_;
    }
    virtual bool report() { return true; }
    virtual bool init() {
        boost::mutex::scoped_lock lock(mutex_);
        if(cm_) return false;

        urdf::Model urdf;
        urdf.initParam("robot_description");
        
        for(HandleMap::iterator it = handles_.begin(); it != handles_.end(); ++it){
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            boost::shared_ptr<const urdf::Joint> joint = urdf.getJoint(it->first);
            if(!joint){
                return false;
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
        cm_.reset(new controller_manager::ControllerManager(this, nh_));
        recover_ = true;
        return true;
    }
    virtual bool recover() {
        boost::mutex::scoped_lock lock(mutex_);
        if(!cm_) return false;
        recover_ = true;
        return true;
    }
    virtual bool shutdown(){
        boost::mutex::scoped_lock lock(mutex_);
        if(cm_) cm_.reset();
        return true;
    }
    void add(const std::string &name, boost::shared_ptr<HandleLayer> handle){
        handles_.insert(std::make_pair(name, handle));
    }

};

class MotorChain : RosChain<ThreadedSocketCANInterface, SharedMaster>{
    boost::shared_ptr< LayerGroup<MotorNode> > motors_;
    boost::shared_ptr< LayerGroup<HandleLayer> > handle_layer_;

    boost::shared_ptr< ControllerManagerLayer> cm_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &module, const boost::shared_ptr<canopen::Node> &node)
    {
        std::string name = module["name"];
        boost::shared_ptr<MotorNode> motor( new MotorNode(node, name + "_motor"));
        motors_->add(motor);

        boost::shared_ptr<HandleLayer> handle( new HandleLayer(name, motor));
        handle_layer_->add(handle);
        cm_->add(name, handle);

        return true;
    }

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv){}
    
    virtual bool setup() {
        motors_.reset( new LayerGroup<MotorNode>("402 Layer"));
        handle_layer_.reset( new LayerGroup<HandleLayer>("Handle Layer"));
        cm_.reset(new ControllerManagerLayer(nh_));

        if(RosChain::setup()){
            boost::mutex::scoped_lock lock(mutex_);
            add(motors_);
            add(handle_layer_);

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

  MotorChain chain(nh, nh_priv);

  if(!chain.setup()){
      return -1;
  }

  ros::waitForShutdown();
  return 0;
}
