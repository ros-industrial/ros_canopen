#include <iostream>
#include <boost/unordered_set.hpp>

#include <socketcan_interface/socketcan.h>

using namespace can;

#include <iostream>

void print_error(const State & s);

void print_frame(const Frame &f){
    
    if(f.is_error){
        std::cout << "E " << std::hex << f.id << std::dec;
    }else if(f.is_extended){
        std::cout << "e " << std::hex << f.id << std::dec;
    }else{
        std::cout << "s " << std::hex << f.id << std::dec;
    }
    
    std::cout << "\t";
    
    if(f.is_rtr){
        std::cout << "r";
    }else{
        std::cout << (int) f.dlc << std::hex;
        
        for(int i=0; i < f.dlc; ++i){
            std::cout << std::hex << " " << (int) f.data[i];
        }
    }
    
    std::cout << std::dec << std::endl;
}

SocketCANInterface driver;

void print_error(const State & s){
    std::string err;
    driver.translateError(s.internal_error,err);
    std::cout << "ERROR: state=" << s.driver_state << " internal_error=" << s.internal_error << "('" << err << "') asio: " << s.error_code << std::endl;
}


int main(int argc, char *argv[]){
    
    if(argc <2 ){
        std::cout << "usage: "<< argv[0] << " DEVICE" << std::endl;
        return 1;
    }
    CommInterface::FrameListener::Ptr frame_printer = driver.createMsgListener(print_frame);
    StateInterface::StateListener::Ptr error_printer = driver.createStateListener(print_error);
    
    if(!driver.init(argv[1],0)){
        print_error(driver.getState());
        return 1;
    }

    driver.run();
    
    return 0;
    
}