#include <ipa_can_interface/dispatcher.h>
#include <ipa_can_interface/socketcan.h>
#include <ipa_canopen_chain_ros/chain_ros.h>

using namespace ipa_can;
using namespace ipa_canopen;

int main(int argc, char** argv){
  ros::init(argc, argv, "ipa_canopen_chain_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  
  RosChain<ThreadedSocketCANInterface, LocalMaster> chain(nh, nh_priv);
  
  if(!chain.setup()){
      return -1;
  }
  
  ros::spin();
  return 0;
}
