#include <ipa_can_interface/dispatcher.h>
#include <ipa_can_interface/socketcan.h>
#include <ipa_canopen_chain_ros/chain_ros.h>

//#include <ipa_canopen_402/ipa_canopen_402.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

// Dummy class
class Node_402 : public ipa_canopen::SimpleLayer{
    boost::shared_ptr <ipa_canopen::Node> n_;
    volatile bool running;
    ipa_canopen::ObjectStorage::Entry<int32_t> actual_pos;

public:
    Node_402(boost::shared_ptr <ipa_canopen::Node> n) : SimpleLayer("Node_402"), n_(n) {
        n->getStorage()->entry(actual_pos, 0x6064);
    }
    virtual bool read() { return true; }
    virtual bool write() { return true; }
    virtual bool report() { return true; }
    virtual bool init() { running = true; return true; }
    virtual bool recover()  { return true; }
    virtual bool shutdown() { running = false; return true; }
    const double getActualPos() { return running?actual_pos.get():0; }
    const double getActualVel() { return 0.0; }
    const double getActualEff() { return 0.0; }
    void setTargetPos(const double &v) {}
    void setTargetVel(const double &v) {}
    void setTargetEff(const double &v) {}
    const double getTargetPos() { return 0.0; }
    const double getTargetVel() { return 0.0; }
    const double getTargetEff() { return 0.0; }
};

using namespace ipa_can;
using namespace ipa_canopen;

class ControllerManagerLayer : public SimpleLayer {
    controller_manager::ControllerManager cm_;
    ros::Timer timer_;
    boost::mutex mutex_;
    bool running_;
    bool recover_;
    ros::Time last_time_;
    
    void timer_func(const ros::TimerEvent& e){
        boost::mutex::scoped_lock lock(mutex_);
        update();
    }
    void update(){
        ros::Time now = ros::Time::now();
        cm_.update(now, now -last_time_, recover_);
        recover_ = false;
        last_time_ = now;
    }
public:
    ControllerManagerLayer(double rate, ros::NodeHandle &nh, hardware_interface::RobotHW *robot)
    :SimpleLayer("ControllerManager"), cm_(robot), timer_(nh.createTimer(ros::Rate(rate), &ControllerManagerLayer::timer_func, this)), running_(false), recover_(false), last_time_(ros::Time::now()) {}
    
    virtual bool read() { 
        boost::mutex::scoped_lock lock(mutex_);
        return running_; 
    }
    virtual bool write()  {
        boost::mutex::scoped_lock lock(mutex_);
        if(running_){
            update();
        }
        return running_; 
    }
    virtual bool report() { return true; }
    virtual bool init() {
        boost::mutex::scoped_lock lock(mutex_);
        if(running_) return false;
        timer_.stop(); 
        running_ = true;
        recover_ = true;
        return true; 
    }
    virtual bool recover() {
        boost::mutex::scoped_lock lock(mutex_);
        if(!running_) return false;
        recover_ = true;
        return true; 
    }
    virtual bool shutdown(){
        boost::mutex::scoped_lock lock(mutex_);
        if(running_) return false;
        timer_.start(); 
        running_ = false;
        return true; 
    }
};

template <typename T> class JointHandleWriter : public hardware_interface::JointHandle {
    double value;
    T & obj_;
    void (T::*writer_)(const double &);
    const double (T::*reader_)(void);
public:
    JointHandleWriter(const hardware_interface::JointStateHandle &jsh, T&  obj, void (T::*writer)(const double &), const double (T::*reader)(void))
    : JointHandle(jsh, &value), obj_(obj), writer_(writer), reader_(reader) {}
    void write() { (obj_.*writer_)(value); }
    void read() { value = (obj_.*reader_)(); }
};

class HandleLayer: public SimpleLayer{
    boost::shared_ptr<Node_402> motor_;
    double pos, vel, eff;
    hardware_interface::JointStateHandle jsh;
    typedef JointHandleWriter<Node_402> CommandWriter;
    typedef boost::unordered_map< const std::string, boost::shared_ptr<CommandWriter> > CommandMap;
    CommandMap commands_;
    
    template <typename T> void addHandle( T &iface, void (Node_402::*writer)(const double &), const double (Node_402::*reader)(void)){
        boost::shared_ptr<CommandWriter> jhw (new CommandWriter(jsh, *motor_, writer, reader));
        commands_[hardware_interface::internal::demangledTypeName<T>()] = jhw;
        iface.registerHandle(*jhw);
        jhw_ = jhw; //TODO: remove
    }
    boost::shared_ptr<CommandWriter> jhw_; 
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<Node_402> & motor)
    : SimpleLayer(name + " Handle"), motor_(motor), jsh(name, &pos, &vel, &eff) {}
    
    void registerHandle(hardware_interface::JointStateInterface &iface){
        iface.registerHandle(jsh);
    }
    void registerHandle(hardware_interface::PositionJointInterface &iface){
       // if pos mode supported
       addHandle(iface, &Node_402::setTargetPos, &Node_402::getTargetPos);
    }
    void registerHandle(hardware_interface::VelocityJointInterface &iface){
       // if vel mode supported
       // addHandle(iface,&Node_402::setTargetVel, &Node_402::getTargetVel);
    }
    void registerHandle(hardware_interface::EffortJointInterface &iface){
       // if eff mode supported
       // addHandle(iface,&Node_402::setTargetEff, &Node_402::getTargetEff);
    }
    void setTargetInterface(const std::string &name){
        CommandMap::iterator it = commands_.find(name);
        if(it != commands_.end()) jhw_ = it->second;
        else jhw_.reset();
    }
    virtual bool read() { 
        bool okay = true;
        // okay = motor.okay();
        if(okay){
            pos = motor_->getActualPos();
            vel = motor_->getActualVel();
            eff = motor_->getActualEff();
            if(jhw_) jhw_->read();
        }
        return okay;
    }
    virtual bool write() {
        if(jhw_){
            jhw_->write();
            return true;
        }
        return false;
    }
    virtual bool report() { return true; }
    virtual bool init() {
        return true; 
    }
    virtual bool recover() {
        return true; 
    }
    virtual bool shutdown(){
        return true; 
    }
};

class MotorChain : RosChain<ThreadedSocketCANInterface, SharedMaster>{
    boost::shared_ptr< LayerGroup<Node_402> > motors_;
    boost::shared_ptr< LayerGroup<HandleLayer> > handles_;
    boost::shared_ptr< ControllerManagerLayer> cm_;
    
    hardware_interface::RobotHW robot_;
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface pos_interface_;
    hardware_interface::VelocityJointInterface vel_interface_;
    hardware_interface::EffortJointInterface eff_interface_;
    
    virtual bool nodeAdded(XmlRpc::XmlRpcValue &module, const boost::shared_ptr<ipa_canopen::Node> &node) {
        boost::shared_ptr<Node_402> motor( new Node_402(node));
        motors_->add(motor);
        
        boost::shared_ptr<HandleLayer> handle( new HandleLayer(module["name"], motor));
        handle->registerHandle(state_interface_);
        handle->registerHandle(pos_interface_);
        handle->registerHandle(vel_interface_);
        handle->registerHandle(eff_interface_);

        handles_->add(handle);
        return true;
    }
    
public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv){}
    
    virtual bool setup() {
        motors_.reset( new LayerGroup<Node_402>("402 Layer"));
        handles_.reset( new LayerGroup<HandleLayer>("Handle Layer"));
        
        if(RosChain::setup()){
            boost::mutex::scoped_lock lock(mutex_);
            add(motors_);
            add(handles_);
            
            robot_.registerInterface(&state_interface_);
            robot_.registerInterface(&pos_interface_);
            robot_.registerInterface(&vel_interface_);
            robot_.registerInterface(&eff_interface_);
            
            ros::NodeHandle chain_handle(nh_,chain_name_);
            cm_.reset(new ControllerManagerLayer(100, chain_handle, &robot_));
            add(cm_);
            
            return true;
        }
        
        return false;
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ipa_canopen_chain_ros_node");
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
