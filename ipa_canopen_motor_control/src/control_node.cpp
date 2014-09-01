#include <ipa_can_interface/dispatcher.h>
#include <ipa_socketcan_driver/socketcan.h>
#include <ipa_canopen_chain_ros/chain_ros.h>

#include <ipa_canopen_402/ipa_canopen_402.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

using namespace ipa_can;
using namespace ipa_canopen;

const double SCALE = 360000 / (2*3.14);
class ScaleNode: public Node_402 {
public:
    ScaleNode(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id, const boost::shared_ptr<SyncProvider> sync = boost::shared_ptr<SyncProvider>())
    :Node_402(interface, dict, node_id, sync) {}
    void getActualPos(double &val){
        val = Node_402::getActualPos() / SCALE;
    }
    void getActualVel(double &val){
        val = Node_402::getActualVel() / SCALE;
    }
   void setTargetPos(const double &pos) {
        Node_402::setTargetPos(int32_t( pos*SCALE));
    }
    
};


class ChainRobot: public RosChain<ScaleNode, DispatchedInterface<SocketCANDriver>, LocalMaster>, public hardware_interface::RobotHW {
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
            this->nodes_->call(&ScaleNode::getActualPos, pos);
            this->nodes_->call(&ScaleNode::getActualVel, vel);
            return true;
        }
        return false;
    }
    bool write(){
        if(this->nodes_){
            this->nodes_->call(&ScaleNode::setTargetPos, pos_cmd);
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
        
        return true;
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
