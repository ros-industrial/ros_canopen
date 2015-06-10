#include <class_loader/class_loader.h>
#include <canopen_402/motor.h>

boost::shared_ptr<canopen::MotorBase> canopen::Motor402::Allocator::allocate(const std::string &name, boost::shared_ptr<canopen::ObjectStorage> storage, const canopen::Settings &settings) {
    return boost::make_shared<canopen::Motor402>(name, storage, settings);
}

CLASS_LOADER_REGISTER_CLASS(canopen::Motor402::Allocator, canopen::MotorBase::Allocator);
