#include <canopen_master/bcm_sync.h>
#include <socketcan_interface/string.h>
#include <iostream>

int main(int argc, char** argv){

    if(argc < 4){
        std::cout << "Usage: " << argv[0] << " DEVICE PERIOD_MS HEADER [OVERFLOW] [+ID*] [-ID*] [--]" << std::endl;
        return 1;
    }

    std::string can_device = argv[1];
    int sync_ms = atoi(argv[2]);
    can::Header header = can::toheader(argv[3]);

    if(!header.isValid()){
            std::cout << "header is invalid" << std::endl;
            return 1;
    }
    int sync_overflow = 0;

    int start = 4;
    if(argc > start && argv[start][0] != '-' && argv[start][0] != '+'){
        sync_overflow = atoi(argv[4]);
        if(sync_overflow == 1 || sync_overflow < 0 || sync_overflow > 240){
            std::cout << "sync overflow is invalid" << std::endl;
            return 1;
        }
        ++start;
    }

    std::set<int> monitored, ignored;

    for(; argc > start; ++start){
        if(strncmp("--", argv[start], 2) == 0) break;
        int id = atoi(argv[start]);

        if(id > 0 && id < 128) monitored.insert(id);
        else if (id < 0 && id > -128) ignored.insert(-id);
        else{
            std::cout << "ID is invalid: " << id  << std::endl;
            return 1;
        }
    }

    can::SocketCANDriverSharedPtr driver = std::make_shared<can::SocketCANDriver>();
    if(!driver->init(can_device, false, can::NoSettings::create())){
        std::cout << "Could not initialize CAN" << std::endl;
        return 1;
    }

    canopen::SyncProperties sync_properties(header, sync_ms, sync_overflow);
    canopen::BCMsync sync(can_device, driver, sync_properties);

    sync.setMonitored(monitored);
    sync.setIgnored(ignored);

    canopen::LayerStatus status;
    sync.init(status);
    if(!status.bounded<canopen::LayerStatus::Warn>()){
        std::cout << "Could not initialize sync" << std::endl;
        return 1;
    }

    driver->run();

    return 0;
}
