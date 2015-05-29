#include <class_loader/class_loader.h>
#include <canopen_master/master.h>

boost::shared_ptr<canopen::Master> canopen::LocalMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::LocalMaster>(interface);
}
boost::shared_ptr<canopen::Master> canopen::SharedMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::SharedMaster>(name, interface);
}

boost::shared_ptr<canopen::Master> canopen::UnrestrictedMaster::Allocator::allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) {
    return boost::make_shared<canopen::UnrestrictedMaster>(name, interface);
}

CLASS_LOADER_REGISTER_CLASS(canopen::LocalMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::SharedMaster::Allocator, canopen::Master::Allocator);
CLASS_LOADER_REGISTER_CLASS(canopen::UnrestrictedMaster::Allocator, canopen::Master::Allocator);
