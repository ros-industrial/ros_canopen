#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H
#include <string>

#include "lely/coapp/master.hpp"
#include "lely/coapp/driver.hpp"

namespace ros2_canopen
{

/**
 * @brief Motor Base Class
 * 
 */
class MotorBase {
protected:
    MotorBase() {}
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

    /**
     * @brief Set target
     * 
     * @param [in] val      Target value
     * @return true 
     * @return false 
     */
    virtual bool setTarget(double val) = 0;

    /**
     * @brief Enter Operation Mode
     * 
     * @param [in] mode     Target Mode 
     * @return true 
     * @return false 
     */
    virtual bool enterModeAndWait(uint16_t mode) = 0;

    /**
     * @brief Check if Operation Mode is supported
     * 
     * @param [in] mode     Operation Mode to be checked 
     * @return true 
     * @return false 
     */
    virtual bool isModeSupported(uint16_t mode) = 0;

    /**
     * @brief Get current Mode
     * 
     * @return uint16_t 
     */
    virtual uint16_t getMode() = 0;

    /**
     * @brief Register default Operation Modes
     * 
     */
    virtual void registerDefaultModes() {}

    typedef std::shared_ptr<MotorBase> MotorBaseSharedPtr;

};
typedef MotorBase::MotorBaseSharedPtr MotorBaseSharedPtr;

}

#endif
