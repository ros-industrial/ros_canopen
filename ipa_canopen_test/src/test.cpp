#include <ipa_canopen_master/canopen.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <ipa_can_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <ipa_socketcan_driver/socketcan.h>

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
            std::cout << "new thread" << std::endl;
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
        }else{
            std::cout << "no thread" << std::endl;
        }
    }
    virtual ~ThreadedInterface() {}
};

boost::shared_ptr<ipa_can::Interface> driver = boost::make_shared<ThreadedInterface< DispatchedInterface<SocketCANDriver> > > ();

void print_frame(const Frame &f){
    std::cout << "in: " << std:: hex << f.id << std::dec << std::endl;
}

int main(int argc, char *argv[]){
    
    if(argc < 3){
        std::cout << "Usage: " << argv[0] << " DEVICE EDS/DCF" << std::endl;
        return -1;
    }
    
    if(!driver->init(argv[1],0)){
        std::cout << "noinit" << std::endl;
        return -1;
    }
    
    sleep(1.0);

    boost::shared_ptr<ipa_canopen::ObjectDict>  dict = ipa_canopen::ObjectDict::fromFile(argv[2]);

    
    Interface::FrameListener::Ptr printer = driver->createMsgListener(print_frame); // printer for all incoming messages
    
    Node node(driver, dict, 1);
    ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  sw;
    node.getStorage()->entry(sw, 0x6041);
    // node.start();
    
    sleep(1.0);
    
    std::cout << "SW2: " << sw.get(false) << std::endl;

    driver->run();
    
    
    return 0;
}
