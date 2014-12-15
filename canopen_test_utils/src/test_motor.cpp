#include <canopen_master/canopen.h>
#include <canopen_master/master.h>
#include <canopen_master/can_layer.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <socketcan_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <socketcan_interface/socketcan.h>

#include <boost/thread.hpp>

#include <canopen_402/canopen_402.h>

#include <signal.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>

bool running = true;

void my_handler(int s){
  printf("Caught signal %d\n",s);
  running = false;
}

using namespace can;
using namespace canopen;

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

int main(int argc, char *argv[])
{

  if(argc < 4){
    std::cout << "Usage: " << argv[0] << " DEVICE EDS/DCF [sync_ms] ID" << std::endl;
    return -1;
  }

  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);

  StateInterface::StateListener::Ptr sprinter = driver->createStateListener(print_state); // printer for all incoming messages

  int sync_ms = 10;
  int id;

  if(argc > 3)
  {
    sync_ms = atol(argv[3]);
    id = atoi(argv[4]);
  }
  std::cout << "ID" << id;

  sleep(1.0);

  boost::shared_ptr<canopen::ObjectDict>  dict = canopen::ObjectDict::fromFile(argv[2]);

  LocalMaster master(argv[1], driver);
  boost::shared_ptr<SyncLayer> sync = master.getSync(SyncProperties(can::MsgHeader(0x80), boost::posix_time::milliseconds(sync_ms), 0));

  boost::shared_ptr<canopen::Node> node (new Node(driver, dict, id, sync));

  std::string name = "402";
  boost::shared_ptr<Node_402> motor( new Node_402(node, name));


  LayerStack stack("test402");
  stack.add(boost::make_shared<CANLayer<ThreadedSocketCANInterface > >(driver, argv[1], 0));
  stack.add(sync);
  stack.add(node);
  stack.add(motor);
  LayerStatus es;

  stack.init(es);
  LayerStatus s;
  canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x007>::type >  sup_mod;
  node->getStorage()->entry(sup_mod, 0x6502);

  LOG("modes: " << sup_mod.get());

  if(sync){
    LayerStatus r,w;
    sync->read(r);
    sync->write(w);
  }

  bool flag_op = false;
  int count = 0;

  while(running)
  {
    LayerStatus r,w;
    stack.read(r);
    stack.write(w);
    boost::this_thread::interruption_point();
    //    if(motor->getState() == motor->Ready_To_Switch_On)
    //    {
    //      break;
    //    }
    if(count > 10000000)
    {

      count = 0;
      flag_op = !flag_op;
    }
    ////        if(flag_op)
    ////        {
    ////          //LOG("Current mode:" << (int)motor->getMode() << " Count: " << count << "Current Pos: " << motor->getActualPos());
    ////          motor->enterMode(motor->Profiled_Velocity);
    ////          motor->setTargetVel(-360000);
    ////        }
    ////        else
    ////        {
    ////         // LOG("Current mode:" << (int)motor->getMode() << " Count: " << count << "Current Pos: " << motor->getActualPos());
    ////          motor->setTargetPos(1000);
    ////          motor->enterMode(motor->Profiled_Position);
    ////        }
    LOG("Current mode:" << (int)motor->getMode() << " Count: " << count << "Current Pos: " << motor->getActualPos() << "Target Pos" << motor->getTargetPos());
    if(count==1)
    {
      motor->setTargetPos((motor->getActualPos()));
      motor->enterMode(motor->Interpolated_Position);
    }
    motor->setTargetPos((motor->getActualPos()-100));
    //motor->setTargetPos((motor->getActualPos()+10));
    count++;
  }

  motor->turnOff();

  stack.shutdown(s);

  return 0;
}

