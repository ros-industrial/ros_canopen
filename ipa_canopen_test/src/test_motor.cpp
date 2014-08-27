#include <ipa_canopen_master/canopen.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <ipa_can_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <ipa_socketcan_driver/socketcan.h>

#include <boost/thread.hpp>

#include <ipa_canopen_402/ipa_canopen_402.h>


using namespace ipa_can;
using namespace ipa_canopen;
using namespace ipa_canopen_402;


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


    Motor motor(node.getStorage());  // 402 !!!!!!!!!!!

    Node::StateListener::Ptr nsl = node.addStateListener(print_node_state);

    node.reset();
    node.start();

    motor.enterMode(motor.Profiled_Velocity);
    motor.init();

    sleep(1.0);

    motor.setTargetVel(-3600000); // 402 !!!!!!!!!!!

    while(true){
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        LOG("Velocity: " <<  motor.getActualVel());  // 402 !!!!!!!!!!!
        LOG("Pos: " <<  motor.getActualPos());  // 402 !!!!!!!!!!!
    }
    driver->run();


    return 0;
}

