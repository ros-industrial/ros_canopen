#include <ipa_canopen_master/canopen.h>
#include <ipa_canopen_master/master.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <ipa_can_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <ipa_can_interface/socketcan.h>

#include <boost/thread.hpp>

#include <ipa_canopen_402/ipa_canopen_402.h>


using namespace ipa_can;
using namespace ipa_canopen;

boost::shared_ptr<ThreadedInterface<SocketCANInterface> > driver = boost::make_shared<ThreadedInterface<SocketCANInterface> > (true);

void print_frame(const Frame &f){
  LOG( "in: " << std:: hex << f.id << std::dec);
}
void print_tpdo(const Frame &f){
  LOG( "TPDO: " << std:: hex << f.id << std::dec);
}

void print_state(const State &f){
  LOG("STATE");
}

void print_node_state(const Node::State &s){
  LOG("NMT:" << s);
}

int main(int argc, char *argv[]){

  if(argc < 3){
    std::cout << "Usage: " << argv[0] << " DEVICE EDS/DCF [sync_ms]" << std::endl;
    return -1;
  }

  // Interface::FrameListener::Ptr printer = driver->createMsgListener(print_frame); // printer for all incoming messages
  // Interface::FrameListener::Ptr tprinter = driver->createMsgListener(Header(0x181), print_tpdo); // printer for all incoming messages
  StateInterface::StateListener::Ptr sprinter = driver->createStateListener(print_state); // printer for all incoming messages

  int sync_ms = 10;
  if(argc > 3) sync_ms = atol(argv[3]);

//  if(!driver->init(argv[1],0)){
//    std::cout << "init failed" << std::endl;
//    return -1;
//  }

  sleep(1.0);

  boost::shared_ptr<ipa_canopen::ObjectDict>  dict = ipa_canopen::ObjectDict::fromFile(argv[2]);

  LocalMaster master(argv[1], driver);
  boost::shared_ptr<SyncLayer> sync = master.getSync(SyncProperties(Header(0x80), boost::posix_time::milliseconds(sync_ms), 0));

  boost::shared_ptr<ipa_canopen::Node> node (new Node(driver, dict, 85, sync));

  std::string name = "402";
  boost::shared_ptr<Node_402> motor( new Node_402(node, name));


  LayerStack stack("test");
  stack.add(boost::make_shared<CANLayer<ThreadedSocketCANInterface > >(driver, argv[1], 0));
  stack.add(sync);
  stack.add(node);
  stack.add(motor);
  LayerStatusExtended es;

  stack.init(es);
  LOG("init: " << (int) es.get()<< " " << es.reason());
  LayerStatus s;

  if(sync){
      sync->read(s);
      s.reset();
      sync->write(s);
  }

  LOG("after sync");
  while(true)
  {
    s.reset();
    stack.read(s);
    LOG("read: " << (int) s.get());
    s.reset();
    stack.write(s);
    LOG("write: " << (int) s.get());
    boost::this_thread::interruption_point();
    //motor.switch_mode()
  }
  stack.shutdown(s);
  return 0;
}

