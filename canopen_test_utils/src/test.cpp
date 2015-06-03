#include <canopen_master/canopen.h>
#include <canopen_master/master.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <socketcan_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <boost/thread.hpp>

using namespace can;
using namespace canopen;

boost::shared_ptr<ThreadedInterface<SocketCANInterface> > driver = boost::make_shared<ThreadedInterface<SocketCANInterface> > ();

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

    if(!driver->init(argv[1],true)){
        std::cout << "init failed" << std::endl;
        return -1;
    }

    sleep(1.0);

    boost::shared_ptr<canopen::ObjectDict>  dict = canopen::ObjectDict::fromFile(argv[2]);

    LocalMaster master(driver);
    boost::shared_ptr<SyncLayer> sync = master.getSync(SyncProperties(can::MsgHeader(0x80), sync_ms, 0));

    Node node(driver, dict, 1, sync);

    canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  sw;
    canopen::ObjectStorage::Entry<canopen::ObjectStorage::DataType<0x006>::type >  cw;
    canopen::ObjectStorage::Entry<int8_t >  op_mode;
    canopen::ObjectStorage::Entry<int8_t >  op_mode_disp;

    node.getStorage()->entry(sw, 0x6041);
    node.getStorage()->entry(op_mode, 0x6060);
    node.getStorage()->entry(op_mode_disp, 0x6061);
    node.getStorage()->entry(cw, 0x6040);

    canopen::ObjectStorage::Entry<int32_t > actual_vel = node.getStorage()->entry<int32_t>(0x606C);
    Node::StateListener::Ptr nsl = node.addStateListener(print_node_state);

    node.reset();
    node.start();

    //LOG("Homing: " << (int) node.getStorage()->entry<int8_t>(0x6098).get());

    sleep(1.0);

    op_mode.set(3);
    LOG("Mode: " << (int) op_mode_disp.get());
    cw.set(0x6); // ready to switch on
    cw.set(0x7); // switch on
    cw.set(0xf); // enable operation

    node.getStorage()->entry<int32_t>(0x60ff).set(360000); // set target velocity

    while(true){
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        //LOG("Status: " << sw.get());
        LOG("Velocity: " <<  actual_vel.get());
    }
    driver->run();


    return 0;
}
