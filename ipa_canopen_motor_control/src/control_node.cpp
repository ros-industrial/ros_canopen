#include <ipa_can_interface/dispatcher.h>
#include <ipa_socketcan_driver/socketcan.h>
#include <ipa_canopen_chain_ros/chain_ros.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

using namespace ipa_can;
using namespace ipa_canopen;

class Node402: public Node {
public:
    Node402(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id, const boost::shared_ptr<SyncProvider> sync = boost::shared_ptr<SyncProvider>())
    :Node(interface, dict, node_id, sync) {}
    const double getActualPos(){
        return 0;
    }
    const double getActualVel(){
        return 0;
    }
    void getActualPosRef(double &val){
        val = getActualPos();
    }
    void getActualVelRef(double &val){
        val = getActualVel();
    }

    void setTargetPos(int32_t pos) {}
    void setTargetPosDouble(const double &pos) {
        setTargetPos(int32_t( pos*100000));
    }
    
};

class ChainRobot: public RosChain<Node402, DispatchedInterface<SocketCANDriver>, LocalMaster>, public hardware_interface::RobotHW {
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    
    std::vector<double> pos_cmd;
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> eff;
    
public:
    ChainRobot(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv){    }
    bool read(){
        if(this->nodes_){
            this->nodes_->call(&Node402::getActualPosRef, pos);
            this->nodes_->call(&Node402::getActualVelRef, vel);
            return true;
        }
        return false;
    }
    bool write(){
        if(this->nodes_){
            this->nodes_->call(&Node402::setTargetPosDouble, pos_cmd);
            return true;
        }
        return false;
    }
    bool setup(){
        if(!RosChain::setup()) return false;
        
        XmlRpc::XmlRpcValue modules;
        nh_priv_.getParam("modules", modules);
        
        pos_cmd.resize(modules.size());
        pos.resize(modules.size());
        vel.resize(modules.size());
        eff.resize(modules.size());

        for (int32_t i = 0; i < modules.size(); ++i){
            XmlRpc::XmlRpcValue &module = modules[i];
            std::string name = module["name"];
            
            hardware_interface::JointStateHandle state_handle(name, &pos[i], &vel[i], &eff[i]);
            jnt_state_interface.registerHandle(state_handle);
            
            hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(name), &pos_cmd[i]);            
            jnt_pos_interface.registerHandle(pos_handle);
        }
        registerInterface(&jnt_state_interface);
        registerInterface(&jnt_pos_interface);
    }
};

class Updater{
    controller_manager::ControllerManager &cm_;
    ChainRobot &chain_;
    ros::Timer timer_;
    void update(const ros::TimerEvent& e){
        if(chain_.read()){
            cm_.update(e.current_real, e.last_real - e.current_real);
            chain_.write();
        }
    }
public:
    Updater(double rate, ros::NodeHandle &nh, controller_manager::ControllerManager &cm, ChainRobot &chain)
    :cm_(cm), chain_(chain), timer_(nh.createTimer(ros::Rate(rate), &Updater::update, this)) {}
};

int main(int argc, char** argv){
  ros::init(argc, argv, "ipa_canopen_chain_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  
  ChainRobot chain(nh, nh_priv);
    
  if(!chain.setup()){
      return -1;
  }
  controller_manager::ControllerManager cm(&chain);
  
  Updater updater(100, nh, cm, chain);

  ros::spin();
  return 0;
}
