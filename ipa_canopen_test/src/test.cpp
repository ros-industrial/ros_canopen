#include <ipa_canopen_master/canopen.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <ipa_can_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <ipa_can_interface/socketcan.h>

#include <boost/thread.hpp>

using namespace ipa_can;
using namespace ipa_canopen;


template<typename WrappedInterface> class ThreadedInterface : public WrappedInterface{
    boost::shared_ptr<boost::thread> thread_;
    void run_thread(){
        WrappedInterface::run();
    }
public:
    virtual bool init(const std::string &device, unsigned int bitrate) {
        if(WrappedInterface::init(device, bitrate)){
            thread_ = boost::make_shared<boost::thread>(&ThreadedInterface::run_thread, this);
            return true;
        }
        return false;
    }
    virtual void shutdown(){
        WrappedInterface::shutdown();
        if(thread_){
            thread_->join();
            thread_.reset();
        }
    }
    virtual void run(){
        if(thread_){
            thread_->join();
        }
    }
    virtual ~ThreadedInterface() {}
    ThreadedInterface(bool loopback = false) : WrappedInterface(loopback) {}
};

boost::shared_ptr<ipa_can::Interface> driver = boost::make_shared<ThreadedInterface< DispatchedInterface<SocketCANDriver> > > (true);

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
    Interface::StateListener::Ptr sprinter = driver->createStateListener(print_state); // printer for all incoming messages

    int sync_ms = 10;
    if(argc > 3) sync_ms = atol(argv[3]);
    
    if(!driver->init(argv[1],0)){
        std::cout << "init failed" << std::endl;
        return -1;
    }
    
    sleep(1.0);

    boost::shared_ptr<ipa_canopen::ObjectDict>  dict = ipa_canopen::ObjectDict::fromFile(argv[2]);
    
    LocalMaster master(driver);
    boost::shared_ptr<SyncProvider> sync = master.getSync(Header(0x80), boost::posix_time::milliseconds(sync_ms), 0);
    
    Node node(driver, dict, 1, sync);

    ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  sw;
    ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  cw;
    ipa_canopen::ObjectStorage::Entry<int8_t >  op_mode;
    ipa_canopen::ObjectStorage::Entry<int8_t >  op_mode_disp;
    
    node.getStorage()->entry(sw, 0x6041);
    node.getStorage()->entry(op_mode, 0x6060);
    node.getStorage()->entry(op_mode_disp, 0x6061);
    node.getStorage()->entry(cw, 0x6040);
    
    ipa_canopen::ObjectStorage::Entry<int32_t > actual_vel = node.getStorage()->entry<int32_t>(0x606C);
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
