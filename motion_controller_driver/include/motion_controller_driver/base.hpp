#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H
#include <string>

#include "lely/coapp/master.hpp"
#include "lely/coapp/driver.hpp"

namespace canopen_402
{

class MotorBase {
protected:
    MotorBase(const std::string &name) {}
public:
    enum OperationMode
    {
        No_Mode = 0,
        Profiled_Position = 1,
        Velocity = 2,
        Profiled_Velocity = 3,
        Profiled_Torque = 4,
        Reserved = 5,
        Homing = 6,
        Interpolated_Position = 7,
        Cyclic_Synchronous_Position = 8,
        Cyclic_Synchronous_Velocity = 9,
        Cyclic_Synchronous_Torque = 10,
    };
    virtual bool setTarget(double val) = 0;
    virtual bool enterModeAndWait(uint16_t mode) = 0;
    virtual bool isModeSupported(uint16_t mode) = 0;
    virtual uint16_t getMode() = 0;
    virtual void registerDefaultModes() {}

    typedef std::shared_ptr<MotorBase> MotorBaseSharedPtr;

};
typedef MotorBase::MotorBaseSharedPtr MotorBaseSharedPtr;

}

#endif