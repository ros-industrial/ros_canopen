
/**
 * Trying to port cia402 stuff from roscanopen...
 * 
 */
#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H

#include <map>
#include <inttypes.h>

//#include <canopen_master/canopen.h>

// namespace canopen
// {

// class MotorBase : public canopen::Layer {
// protected:
//     MotorBase(const std::string &name) : Layer(name) {}
// public:
//     enum OperationMode
//     {
//         No_Mode = 0,
//         Profiled_Position = 1,
//         Velocity = 2,
//         Profiled_Velocity = 3,
//         Profiled_Torque = 4,
//         Reserved = 5,
//         Homing = 6,
//         Interpolated_Position = 7,
//         Cyclic_Synchronous_Position = 8,
//         Cyclic_Synchronous_Velocity = 9,
//         Cyclic_Synchronous_Torque = 10,
//     };
//     virtual bool setTarget(double val) = 0;
//     virtual bool enterModeAndWait(uint16_t mode) = 0;
//     virtual bool isModeSupported(uint16_t mode) = 0;
//     virtual uint16_t getMode() = 0;
//     virtual void registerDefaultModes(ObjectStorageSharedPtr storage) {}

//     typedef std::shared_ptr<MotorBase> MotorBaseSharedPtr;

//     class Allocator {
//     public:
//         virtual MotorBaseSharedPtr allocate(const std::string &name, ObjectStorageSharedPtr storage, const canopen::Settings &settings) = 0;
//         virtual ~Allocator() {}
//     };
// };
// typedef MotorBase::MotorBaseSharedPtr MotorBaseSharedPtr;


class State402
{
public:
    enum StatusWord
    {
        SW_Ready_To_Switch_On = 0,
        SW_Switched_On = 1,
        SW_Operation_enabled = 2,
        SW_Fault = 3,
        SW_Voltage_enabled = 4,
        SW_Quick_stop = 5,
        SW_Switch_on_disabled = 6,
        SW_Warning = 7,
        SW_Manufacturer_specific0 = 8,
        SW_Remote = 9,
        SW_Target_reached = 10,
        SW_Internal_limit = 11,
        SW_Operation_mode_specific0 = 12,
        SW_Operation_mode_specific1 = 13,
        SW_Manufacturer_specific1 = 14,
        SW_Manufacturer_specific2 = 15
    };
    enum InternalState
    {
        Unknown = 0,
        Start = 0,
        Not_Ready_To_Switch_On = 1,
        Switch_On_Disabled = 2,
        Ready_To_Switch_On = 3,
        Switched_On = 4,
        Operation_Enable = 5,
        Quick_Stop_Active = 6,
        Fault_Reaction_Active = 7,
        Fault = 8,
    };
    InternalState getState();
    InternalState read(uint16_t sw);
    State402() : state_(Unknown) {}

private:
    InternalState state_;
};

class Command402
{
    struct Op
    {
        uint16_t to_set_;
        uint16_t to_reset_;
        Op(uint16_t to_set, uint16_t to_reset) : to_set_(to_set), to_reset_(to_reset) {}
        void operator()(uint16_t &val) const
        {
            val = (val & ~to_reset_) | to_set_;
        }
    };
    class TransitionTable
    {
        std::map<std::pair<State402::InternalState, State402::InternalState>, Op> transitions_;
        void add(const State402::InternalState &from, const State402::InternalState &to, Op op)
        {
            transitions_.insert(std::make_pair(std::make_pair(from, to), op));
        }

    public:
        TransitionTable();
        const Op &get(const State402::InternalState &from, const State402::InternalState &to) const
        {
            return transitions_.at(std::make_pair(from, to));
        }
    };
    static const TransitionTable transitions_;
    static State402::InternalState nextStateForEnabling(State402::InternalState state);
    Command402();

public:
    enum ControlWord
    {
        CW_Switch_On = 0,
        CW_Enable_Voltage = 1,
        CW_Quick_Stop = 2,
        CW_Enable_Operation = 3,
        CW_Operation_mode_specific0 = 4,
        CW_Operation_mode_specific1 = 5,
        CW_Operation_mode_specific2 = 6,
        CW_Fault_Reset = 7,
        CW_Halt = 8,
        CW_Operation_mode_specific3 = 9,
        // CW_Reserved1=10,
        CW_Manufacturer_specific0 = 11,
        CW_Manufacturer_specific1 = 12,
        CW_Manufacturer_specific2 = 13,
        CW_Manufacturer_specific3 = 14,
        CW_Manufacturer_specific4 = 15,
    };

    static bool setTransition(uint16_t &cw, const State402::InternalState &from, const State402::InternalState &to, State402::InternalState *next);
};

template <uint16_t MASK>
class WordAccessor
{
    uint16_t &word_;

public:
    WordAccessor(uint16_t &word) : word_(word) {}
    bool set(uint8_t bit)
    {
        uint16_t val = MASK & (1 << bit);
        word_ |= val;
        return val;
    }
    bool reset(uint8_t bit)
    {
        uint16_t val = MASK & (1 << bit);
        word_ &= ~val;
        return val;
    }
    bool get(uint8_t bit) const { return word_ & (1 << bit); }
    uint16_t get() const { return word_ & MASK; }
    WordAccessor &operator=(const uint16_t &val)
    {
        uint16_t was = word_;
        word_ = (word_ & ~MASK) | (val & MASK);
        return *this;
    }
};

#endif