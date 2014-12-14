#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/chain_ros.h>

using namespace ipa_can;
using namespace ipa_canopen;

int main(int argc, char** argv){
  ros::init(argc, argv, "canopen_chain_node_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  
  RosChain<ThreadedSocketCANInterface, SharedMaster> chain(nh, nh_priv);
  
  if(!chain.setup()){
      return -1;
  }
  
  ros::spin();
  return 0;
}
