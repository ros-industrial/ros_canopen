#include <ipa_canopen_master/canopen.h>
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

    if(!driver->init(argv[1],0)){
        std::cout << "init failed" << std::endl;
        return -1;
    }

    sleep(1.0);

    boost::shared_ptr<ipa_canopen::ObjectDict>  dict = ipa_canopen::ObjectDict::fromFile(argv[2]);

    LocalMaster master(driver);
    boost::shared_ptr<SyncProvider> sync = master.getSync(Header(0x80), boost::posix_time::milliseconds(sync_ms), 0);

    Node node(driver, dict, 1, sync);


    Node_402 motor(driver, dict, 1, sync);  // 402 !!!!!!!!!!!

    Node::StateListener::Ptr nsl = node.addStateListener(print_node_state);

    node.reset();
    node.start();

    motor.init();

    sleep(1.0);

    bool flag_op = false;

    while(true)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        motor.turnOff();

        if(flag_op)
        {
            LOG("Entering Velocity OP mode");  // 402 !!!!!!!!!!!
            motor.enterMode(motor.Profiled_Velocity);
            motor.turnOn();
            motor.setTargetVel(-360000); // 402 !!!!!!!!!!!
            motor.operate();
            std::cout << "Mode:" << static_cast<int>(motor.getMode()) << std::endl;
            LOG("Velocity: " <<  motor.getActualVel());  // 402 !!!!!!!!!!!
        }
        else
        {
            LOG("Entering Position OP mode");  // 402 !!!!!!!!!!!
            motor.enterMode(motor.Profiled_Position);
            motor.turnOn();
            motor.setTargetPos(100000); // 402 !!!!!!!!!!!
            motor.operate();

            std::cout << "Mode:" << static_cast<int>(motor.getMode()) << std::endl;
            LOG("Pos: " <<  motor.getActualInternalPos());  // 402 !!!!!!!!!!!
            LOG("Actual Pos: " <<  motor.getActualInternalPos());  // 402 !!!!!!!!!!!
            sleep(10.0);
        }

        sleep(2.0);

        motor.setTargetVel(0);

        //flag_op = !flag_op;
    }

    driver->run();

    motor.turnOff();

    return 0;
}

