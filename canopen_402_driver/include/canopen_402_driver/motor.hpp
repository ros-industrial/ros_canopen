#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include <limits>
#include <algorithm>
#include <iostream>
#include <bitset>
#include "rclcpp/rclcpp.hpp"

#include "canopen_402_driver/lely_motion_controller_bridge.hpp"
using namespace ros2_canopen;
namespace ros2_canopen
{
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
        bool waitForNewState(const std::chrono::steady_clock::time_point &abstime, InternalState &state);
        State402() : state_(Unknown) {}

    private:
        std::condition_variable cond_;
        std::mutex mutex_;
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
            boost::container::flat_map<std::pair<State402::InternalState, State402::InternalState>, Op> transitions_;
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
            word_ = (word_ & ~MASK) | (val & MASK);
            return *this;
        }
    };

    class Mode
    {
    public:
        const uint16_t mode_id_;
        Mode(uint16_t id) : mode_id_(id) {}
        typedef WordAccessor<(1 << Command402::CW_Operation_mode_specific0) | (1 << Command402::CW_Operation_mode_specific1) | (1 << Command402::CW_Operation_mode_specific2) | (1 << Command402::CW_Operation_mode_specific3)> OpModeAccesser;
        virtual bool start() = 0;
        virtual bool read(const uint16_t &sw) = 0;
        virtual bool write(OpModeAccesser &cw) = 0;
        virtual bool setTarget(const double &val) { return false; }
        virtual ~Mode() {}
    };
    typedef std::shared_ptr<Mode> ModeSharedPtr;

    template <typename T>
    class ModeTargetHelper : public Mode
    {
        T target_;
        std::atomic<bool> has_target_;

    public:
        ModeTargetHelper(uint16_t mode) : Mode(mode) {}
        bool hasTarget() { return has_target_; }
        T getTarget() { return target_; }
        virtual bool setTarget(const double &val)
        {
            if (std::isnan(val))
            {
                std::cout << "canopen_402 target command is not a number" << std::endl;
                return false;
            }

            using boost::numeric_cast;
            using boost::numeric::negative_overflow;
            using boost::numeric::positive_overflow;

            try
            {
                target_ = numeric_cast<T>(val);
            }
            catch (negative_overflow &)
            {
                std::cout << "canopen_402 Command " << val << " does not fit into target, clamping to min limit" << std::endl;
                target_ = std::numeric_limits<T>::min();
            }
            catch (positive_overflow &)
            {
                std::cout << "canopen_402 Command " << val << " does not fit into target, clamping to max limit" << std::endl;
                target_ = std::numeric_limits<T>::max();
            }
            catch (...)
            {
                std::cout << "canopen_402 Was not able to cast command " << val << std::endl;
                return false;
            }

            has_target_ = true;
            return true;
        }
        virtual bool start()
        {
            has_target_ = false;
            return true;
        }
    };

    template <uint16_t ID, typename TYPE, CODataTypes TPY, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK>
    class ModeForwardHelper : public ModeTargetHelper<TYPE>
    {
        std::shared_ptr<LelyMotionControllerBridge> driver;
        std::shared_ptr<RemoteObject> obj;

    public:
        ModeForwardHelper(std::shared_ptr<LelyMotionControllerBridge> driver) : ModeTargetHelper<TYPE>(ID)
        {
            this->obj = driver->create_remote_obj(OBJ, SUB, TPY);
            this->driver = driver;
        }
        virtual bool read(const uint16_t &sw) { return true; }
        virtual bool write(Mode::OpModeAccesser &cw)
        {
            if (this->hasTarget())
            {
                cw = cw.get() | CW_MASK;

                driver->set_remote_obj(obj, this->getTarget());
                return true;
            }
            else
            {
                cw = cw.get() & ~CW_MASK;
                return false;
            }
        }
    };

    typedef ModeForwardHelper<MotorBase::Profiled_Velocity, int32_t, CODataTypes::COData32, 0x60FF, 0, 0> ProfiledVelocityMode;
    typedef ModeForwardHelper<MotorBase::Profiled_Torque, int16_t, CODataTypes::COData16, 0x6071, 0, 0> ProfiledTorqueMode;
    typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Position, int32_t, CODataTypes::COData32, 0x607A, 0, 0> CyclicSynchronousPositionMode;
    typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Velocity, int32_t, CODataTypes::COData32, 0x60FF, 0, 0> CyclicSynchronousVelocityMode;
    typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Torque, int16_t, CODataTypes::COData16, 0x6071, 0, 0> CyclicSynchronousTorqueMode;
    typedef ModeForwardHelper<MotorBase::Velocity, int16_t, CODataTypes::COData16, 0x6042, 0, (1 << Command402::CW_Operation_mode_specific0) | (1 << Command402::CW_Operation_mode_specific1) | (1 << Command402::CW_Operation_mode_specific2)> VelocityMode;
    typedef ModeForwardHelper<MotorBase::Interpolated_Position, int32_t, CODataTypes::COData32, 0x60C1, 0x01, (1 << Command402::CW_Operation_mode_specific0)> InterpolatedPositionMode;

    class ProfiledPositionMode : public ModeTargetHelper<int32_t>
    {
        const uint16_t index = 0x607A;
        std::shared_ptr<LelyMotionControllerBridge> driver;
        std::shared_ptr<RemoteObject> obj;

        double last_target_;
        uint16_t sw_;

    public:
        enum SW_masks
        {
            MASK_Reached = (1 << State402::SW_Target_reached),
            MASK_Acknowledged = (1 << State402::SW_Operation_mode_specific0),
            MASK_Error = (1 << State402::SW_Operation_mode_specific1),
        };
        enum CW_bits
        {
            CW_NewPoint = Command402::CW_Operation_mode_specific0,
            CW_Immediate = Command402::CW_Operation_mode_specific1,
            CW_Blending = Command402::CW_Operation_mode_specific3,
        };
        ProfiledPositionMode(std::shared_ptr<LelyMotionControllerBridge> driver)
            : ModeTargetHelper(MotorBase::Profiled_Position)
        {
            this->driver  = driver;
            obj = driver->create_remote_obj(index, 0U, CODataTypes::COData32);
        }

        virtual bool start()
        {
            sw_ = 0;
            last_target_ = std::numeric_limits<double>::quiet_NaN();
            return ModeTargetHelper::start();
        }
        virtual bool read(const uint16_t &sw)
        {
            sw_ = sw;
            return (sw & MASK_Error) == 0;
        }
        virtual bool write(OpModeAccesser &cw)
        {
            cw.set(CW_Immediate);
            if (hasTarget())
            {
                int32_t target = getTarget();
                if ((sw_ & MASK_Acknowledged) == 0 && target != last_target_)
                {
                    if (cw.get(CW_NewPoint))
                    {
                        cw.reset(CW_NewPoint); // reset if needed
                    }
                    else
                    {
                        driver->set_remote_obj(obj, target);
                        cw.set(CW_NewPoint);
                        last_target_ = target;
                    }
                }
                else if (sw_ & MASK_Acknowledged)
                {
                    cw.reset(CW_NewPoint);
                }
                return true;
            }
            return false;
        }
    };

    class HomingMode : public Mode
    {
    protected:
        enum SW_bits
        {
            SW_Attained = State402::SW_Operation_mode_specific0,
            SW_Error = State402::SW_Operation_mode_specific1,
        };
        enum CW_bits
        {
            CW_StartHoming = Command402::CW_Operation_mode_specific0,
        };

    public:
        HomingMode() : Mode(MotorBase::Homing) {}
        virtual bool executeHoming() = 0;
    };

    class DefaultHomingMode : public HomingMode
    {
        const uint16_t index = 0x6098;
        std::shared_ptr<LelyMotionControllerBridge> driver;
        std::shared_ptr<RemoteObject> obj;

        std::atomic<bool> execute_;

        std::mutex mutex_;
        std::condition_variable cond_;
        uint16_t status_;

        enum SW_masks
        {
            MASK_Reached = (1 << State402::SW_Target_reached),
            MASK_Attained = (1 << SW_Attained),
            MASK_Error = (1 << SW_Error),
        };
        bool error(const std::string &msg)
        {
            execute_ = false;
            std::cout << msg << std::endl;
            return false;
        }

    public:
        DefaultHomingMode(std::shared_ptr<LelyMotionControllerBridge> driver)
        {
            this->driver = driver;
            obj = driver->create_remote_obj(index, 0U, CODataTypes::COData32);
        }
        virtual bool start();
        virtual bool read(const uint16_t &sw);
        virtual bool write(OpModeAccesser &cw);

        virtual bool executeHoming();
    };

    class Motor402 : public MotorBase
    {
    public:
        Motor402(std::shared_ptr<LelyMotionControllerBridge> driver) : 
            MotorBase(),
            switching_state_(State402::Operation_Enable),
            monitor_mode_(true),
            state_switch_timeout_(5)
        {
            this->driver = driver;
            status_word_entry_ = driver->create_remote_obj(status_word_entry_index, 0U, CODataTypes::COData16);
            control_word_entry_ = driver->create_remote_obj(control_word_entry_index, 0U, CODataTypes::COData16);
            op_mode_display_ = driver->create_remote_obj(op_mode_display_index, 0U, CODataTypes::COData8);
            op_mode_ = driver->create_remote_obj(op_mode_index, 0U, CODataTypes::COData8);
            supported_drive_modes_ = driver->create_remote_obj(supported_drive_modes_index, 0U, CODataTypes::COData32);
        }

        virtual bool setTarget(double val);
        virtual bool enterModeAndWait(uint16_t mode);
        virtual bool isModeSupported(uint16_t mode);
        virtual uint16_t getMode();
        bool readState();
        void handleDiag();
        /**
         * @brief Initialise the drive
         * 
         * This function intialises the drive. This means, it first
         * attempts to bring the device to operational state (CIA402)
         * and then executes the chosen homing method.
         * 
         */
        bool handleInit();
        /**
         * @brief Read objects of the drive
         * 
         * This function should be called regularly. It reads the status word
         * from the device and translates it into the devices state.
         * 
         */
        void handleRead();
        /**
         * @brief Writes objects to the drive
         * 
         * This function should be called regularly. It writes the new command
         * word to the drive
         * 
         */
        void handleWrite();
        /**
         * @brief Shutdowns the drive
         * 
         * This function shuts down the drive by bringing it into
         * SwitchOn disbled state.
         * 
         */
        bool handleShutdown();
        /**
         * @brief Executes a quickstop
         * 
         * The function executes a quickstop.
         * 
         */
        bool handleHalt();

        /**
         * @brief Recovers the device from fault
         * 
         * This function tries to reset faults and
         * put the device back to operational state.
         * 
         */
        bool handleRecover();

        /**
         * @brief Register a new operation mode for the drive
         * 
         * This function will register an operation mode for the drive.
         * It will check if the mode is supported by the drive by reading
         * 0x6508 object.
         * 
         * @tparam T 
         * @tparam Args 
         * @param mode 
         * @param args 
         * @return true 
         * @return false 
         */
        template <typename T, typename... Args>
        bool registerMode(uint16_t mode, Args &&...args)
        {
            return mode_allocators_.insert(std::make_pair(mode, [args..., mode, this]()
                                                          {
             if(isModeSupportedByDevice(mode)) registerMode(mode, ModeSharedPtr(new T(args...))); }))
                .second;
        }

        /**
         * @brief Tries to register the standard operation modes defined in cia402
         * 
         */
        virtual void registerDefaultModes()
        {
            registerMode<ProfiledPositionMode>(MotorBase::Profiled_Position, driver);
            registerMode<VelocityMode>(MotorBase::Velocity, driver);
            registerMode<ProfiledVelocityMode>(MotorBase::Profiled_Velocity, driver);
            registerMode<ProfiledTorqueMode>(MotorBase::Profiled_Torque, driver);
            registerMode<DefaultHomingMode>(MotorBase::Homing, driver);
            registerMode<InterpolatedPositionMode>(MotorBase::Interpolated_Position, driver);
            registerMode<CyclicSynchronousPositionMode>(MotorBase::Cyclic_Synchronous_Position, driver);
            registerMode<CyclicSynchronousVelocityMode>(MotorBase::Cyclic_Synchronous_Velocity, driver);
            registerMode<CyclicSynchronousTorqueMode>(MotorBase::Cyclic_Synchronous_Torque, driver);
        }

    private:
        virtual bool isModeSupportedByDevice(uint16_t mode);
        void registerMode(uint16_t id, const ModeSharedPtr &m);

        ModeSharedPtr allocMode(uint16_t mode);

        bool switchMode(uint16_t mode);
        bool switchState(const State402::InternalState &target);

        std::atomic<uint16_t> status_word_;
        uint16_t control_word_;
        std::mutex cw_mutex_;
        std::atomic<bool> start_fault_reset_;
        std::atomic<State402::InternalState> target_state_;

        State402 state_handler_;

        std::mutex map_mutex_;
        std::unordered_map<uint16_t, ModeSharedPtr> modes_;
        typedef std::function<void()> AllocFuncType;
        std::unordered_map<uint16_t, AllocFuncType> mode_allocators_;

        ModeSharedPtr selected_mode_;
        uint16_t mode_id_;
        std::condition_variable mode_cond_;
        std::mutex mode_mutex_;
        const State402::InternalState switching_state_;
        const bool monitor_mode_;
        const std::chrono::seconds state_switch_timeout_;

        std::shared_ptr<LelyMotionControllerBridge> driver;
        std::shared_ptr<RemoteObject> status_word_entry_;
        std::shared_ptr<RemoteObject> control_word_entry_;
        std::shared_ptr<RemoteObject> op_mode_display_;
        std::shared_ptr<RemoteObject> op_mode_;
        std::shared_ptr<RemoteObject> supported_drive_modes_;
        const uint16_t status_word_entry_index = 0x6041;
        const uint16_t control_word_entry_index = 0x6040;
        const uint16_t op_mode_display_index = 0x6061;
        const uint16_t op_mode_index = 0x6060;
        const uint16_t supported_drive_modes_index = 0x6502;
    };

}


#endif
