/**
 * Trying to port cia402 stuff from roscanopen...
 * 
 */

#include <cstdio>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <mutex>
#include <condition_variable>
#include <future>
#include <mutex>
#include <atomic>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace lely;
using namespace std::chrono_literals;

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

    InternalState getState()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }

    InternalState read(uint16_t sw)
    {
        static const uint16_t r = (1 << SW_Ready_To_Switch_On);
        static const uint16_t s = (1 << SW_Switched_On);
        static const uint16_t o = (1 << SW_Operation_enabled);
        static const uint16_t f = (1 << SW_Fault);
        static const uint16_t q = (1 << SW_Quick_stop);
        static const uint16_t d = (1 << SW_Switch_on_disabled);

        InternalState new_state = Unknown;

        uint16_t state = sw & (d | q | f | o | s | r);
        switch (state)
        {
        //   ( d | q | f | o | s | r ):
        case (0 | 0 | 0 | 0 | 0 | 0):
        case (0 | q | 0 | 0 | 0 | 0):
            new_state = Not_Ready_To_Switch_On;
            break;

        case (d | 0 | 0 | 0 | 0 | 0):
        case (d | q | 0 | 0 | 0 | 0):
            new_state = Switch_On_Disabled;
            break;

        case (0 | q | 0 | 0 | 0 | r):
            new_state = Ready_To_Switch_On;
            break;

        case (0 | q | 0 | 0 | s | r):
            new_state = Switched_On;
            break;

        case (0 | q | 0 | o | s | r):
            new_state = Operation_Enable;
            break;

        case (0 | 0 | 0 | o | s | r):
            new_state = Quick_Stop_Active;
            break;

        case (0 | 0 | f | o | s | r):
        case (0 | q | f | o | s | r):
            new_state = Fault_Reaction_Active;
            break;

        case (0 | 0 | f | 0 | 0 | 0):
        case (0 | q | f | 0 | 0 | 0):
            new_state = Fault;
            break;

        default:
            //ROSCANOPEN_WARN("canopen_402", "Motor is currently in an unknown state: " << std::hex <<  state << std::dec);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (new_state != state_)
        {
            state_ = new_state;
            cond_.notify_all();
        }
        return state_;
    }
    /**
     * @brief Wait for state Change
     * 
     * Blocking wait for motion devices state to change. 
     * 
     * @return InternalState 
     */
    InternalState waitForStateChange()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock);
        return state_;
    }

    /**
     * @brief Async wait for State Change
     * 
     * @return std::future<InternalState> 
     */
    std::future<InternalState> asyncWaitForStateChange()
    {
        return std::async(std::launch::async, &State402::waitForStateChange, this);
    }

    /**
     * @brief Wait for specific State
     * 
     * @param target_state 
     * @return InternalState 
     */
    InternalState waitForState(InternalState target_state)
    {
        do
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cond_.wait(lock);
            if (state_ == target_state)
            {
                break;
            }
            lock.unlock();
        } while (state_ != target_state);
        return target_state;
    }

    /**
     * @brief Async wait for specific state
     * 
     * @param target_state 
     * @return std::future<InternalState> 
     */
    std::future<InternalState> asyncWaitForState(InternalState target_state)
    {
        return std::async(std::launch::async, &State402::waitForState, this, target_state);
    }

    State402() : state_(Unknown) {}

private:
    InternalState state_;
    std::mutex mutex_;
    std::condition_variable cond_;
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
        TransitionTable()
        {
            typedef State402 s;

            //transitions_.reserve(32);

            Op disable_voltage(0, (1 << CW_Fault_Reset) | (1 << CW_Enable_Voltage));
            /* 7*/ add(s::Ready_To_Switch_On, s::Switch_On_Disabled, disable_voltage);
            /* 9*/ add(s::Operation_Enable, s::Switch_On_Disabled, disable_voltage);
            /*10*/ add(s::Switched_On, s::Switch_On_Disabled, disable_voltage);
            /*12*/ add(s::Quick_Stop_Active, s::Switch_On_Disabled, disable_voltage);

            Op automatic(0, 0);
            /* 0*/ add(s::Start, s::Not_Ready_To_Switch_On, automatic);
            /* 1*/ add(s::Not_Ready_To_Switch_On, s::Switch_On_Disabled, automatic);
            /*14*/ add(s::Fault_Reaction_Active, s::Fault, automatic);

            Op shutdown((1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage), (1 << CW_Fault_Reset) | (1 << CW_Switch_On));
            /* 2*/ add(s::Switch_On_Disabled, s::Ready_To_Switch_On, shutdown);
            /* 6*/ add(s::Switched_On, s::Ready_To_Switch_On, shutdown);
            /* 8*/ add(s::Operation_Enable, s::Ready_To_Switch_On, shutdown);

            Op switch_on((1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage) | (1 << CW_Switch_On), (1 << CW_Fault_Reset) | (1 << CW_Enable_Operation));
            /* 3*/ add(s::Ready_To_Switch_On, s::Switched_On, switch_on);
            /* 5*/ add(s::Operation_Enable, s::Switched_On, switch_on);

            Op enable_operation((1 << CW_Quick_Stop) | (1 << CW_Enable_Voltage) | (1 << CW_Switch_On) | (1 << CW_Enable_Operation), (1 << CW_Fault_Reset));
            /* 4*/ add(s::Switched_On, s::Operation_Enable, enable_operation);
            /*16*/ add(s::Quick_Stop_Active, s::Operation_Enable, enable_operation);

            Op quickstop((1 << CW_Enable_Voltage), (1 << CW_Fault_Reset) | (1 << CW_Quick_Stop));
            /* 7*/ add(s::Ready_To_Switch_On, s::Quick_Stop_Active, quickstop); // transit to Switch_On_Disabled
            /*10*/ add(s::Switched_On, s::Quick_Stop_Active, quickstop);        // transit to Switch_On_Disabled
            /*11*/ add(s::Operation_Enable, s::Quick_Stop_Active, quickstop);

            // fault reset
            /*15*/ add(s::Fault, s::Switch_On_Disabled, Op((1 << CW_Fault_Reset), 0));
        }

        const Op &get(const State402::InternalState &from, const State402::InternalState &to) const
        {
            return transitions_.at(std::make_pair(from, to));
        }
    };

    static const TransitionTable transitions_;
    static State402::InternalState nextStateForEnabling(State402::InternalState state)
    {
        switch (state)
        {
        case State402::Start:
            return State402::Not_Ready_To_Switch_On;

        case State402::Fault:
        case State402::Not_Ready_To_Switch_On:
            return State402::Switch_On_Disabled;

        case State402::Switch_On_Disabled:
            return State402::Ready_To_Switch_On;

        case State402::Ready_To_Switch_On:
            return State402::Switched_On;

        case State402::Switched_On:
        case State402::Quick_Stop_Active:
        case State402::Operation_Enable:
            return State402::Operation_Enable;

        case State402::Fault_Reaction_Active:
            return State402::Fault;
        }
        throw std::out_of_range("state value is illegal");
    }

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

    static bool setTransition(uint16_t &cw, const State402::InternalState &from, const State402::InternalState &to, State402::InternalState *next)
    {
        try
        {
            if (from != to)
            {
                State402::InternalState hop = to;
                if (next)
                {
                    if (to == State402::Operation_Enable)
                        hop = nextStateForEnabling(from);
                    *next = hop;
                }
                transitions_.get(from, hop)(cw);
            }
            return true;
        }
        catch (...)
        {
        }
        return false;
    }
};

class CIA402_Driver : canopen::FiberDriver
{
public:
    using FiberDriver::FiberDriver;
    CIA402_Driver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id) : FiberDriver(exec, master, id)
    {
    }
    enum CIA402Objects
    {
        Slave_Heartbeat = 0x1017,
        Slave_Status_Word = 0x6041,
        Slave_Control_Word = 0x6040,
        Slave_Operation_Mode = 0x6060,
        Slave_Operation_Mode_D = 0x6061,
        Slave_Actual_Position = 0x6064,
        Slave_Actual_Velocity = 0x606B,
    };
    enum CIA402Return
    {
        SUCCESS,
        ERROR,
        TIMEOUT
    };

private:
    State402 cia402_state_handler_;
    std::atomic<State402::InternalState> cia402_state_;

    void
    OnBoot(canopen::NmtState state, char es,
           const std::string &whatisit) noexcept override
    {
    }

    void
    OnConfig(std::function<void(std::error_code ec)> res) noexcept override
    {
        // Set Heartbeat of Slave
        Wait(AsyncWrite<uint16_t>(0x1017, 0, 1000));
    }

    void
    OnState(canopen::NmtState state) noexcept override
    {
        switch (state)
        {
        case canopen::NmtState::PREOP:
            break;

        case canopen::NmtState::START:
            break;
        }
    }

    void
    OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
    {
        if (idx == CIA402Objects::Slave_Status_Word && subidx == 0)
        {
            //Process status word
            uint16_t val = rpdo_mapped[idx][subidx];
            cia402_state_ = cia402_state_handler_.read(val);
        }
    }

    /**
     * @brief Bring device to operation enable
     * 
     * @return CIA402Return 
     */
    CIA402Return
    enableOperation()
    {
        uint16_t cw;
        State402::InternalState next;
        State402::InternalState reached;

        //As lon as Operation ENable not reached
        while (cia402_state_ != State402::Operation_Enable)
        {       
            //If transition can be set
            if (Command402::setTransition(cw, cia402_state_, State402::Operation_Enable, &next))
            {
                //Write Control Word for now as SDO
                //@Todo: Wrtie Control Word via PDO
                Wait(AsyncWrite<uint16_t>(int(Slave_Control_Word), 0, int(cw)));

                //Wait for State Change
                auto fut = cia402_state_handler_.asyncWaitForState(next);
                if (fut.wait_for(500ms) == std::future_status::ready)
                {
                    //Get the reached state
                    reached = fut.get();
                    //Check that reached state is next state
                    if(reached != next)
                    {
                        //Next state not reached state -> something went wrong
                        return CIA402Return::ERROR;return CIA402Return::ERROR;
                    }
                }
                else
                {  
                    //Timeout occured
                    return CIA402Return::TIMEOUT;
                }
            }
            else
            {
                //Can't go to Operation Enable -> something went wrong
                return CIA402Return::ERROR;
            }
        }
        return CIA402Return::SUCCESS;
    }
};