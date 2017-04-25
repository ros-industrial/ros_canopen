#ifndef CANOPEN_402_MOTOR_H
#define CANOPEN_402_MOTOR_H

#include <canopen_402/base.h>
#include <canopen_master/canopen.h>
#include <boost/function.hpp>
#include <boost/container/flat_map.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <algorithm>

namespace canopen
{

class State402{
public:
    enum StatusWord
    {
        SW_Ready_To_Switch_On=0,
        SW_Switched_On=1,
        SW_Operation_enabled=2,
        SW_Fault=3,
        SW_Voltage_enabled=4,
        SW_Quick_stop=5,
        SW_Switch_on_disabled=6,
        SW_Warning=7,
        SW_Manufacturer_specific0=8,
        SW_Remote=9,
        SW_Target_reached=10,
        SW_Internal_limit=11,
        SW_Operation_mode_specific0=12,
        SW_Operation_mode_specific1=13,
        SW_Manufacturer_specific1=14,
        SW_Manufacturer_specific2=15
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
    bool waitForNewState(const time_point &abstime, InternalState &state);
    State402() : state_(Unknown) {}
private:
    boost::condition_variable cond_;
    boost::mutex mutex_;
    InternalState state_;
};

class Command402 {
    struct Op {
        uint16_t to_set_;
        uint16_t to_reset_;
        Op(uint16_t to_set ,uint16_t to_reset) : to_set_(to_set), to_reset_(to_reset) {}
        void operator()(uint16_t &val) const {
            val = (val & ~to_reset_) | to_set_;
        }
    };
    class TransitionTable {
        boost::container::flat_map< std::pair<State402::InternalState, State402::InternalState>, Op > transitions_;
        void add(const State402::InternalState &from, const State402::InternalState &to, Op op){
            transitions_.insert(std::make_pair(std::make_pair(from, to), op));
        }
    public:
        TransitionTable();
        const Op& get(const State402::InternalState &from, const State402::InternalState &to) const {
            return transitions_.at(std::make_pair(from, to));
        }
    };
    static const TransitionTable transitions_;
    static State402::InternalState nextStateForEnabling(State402::InternalState state);
    Command402();
public:
    enum ControlWord
    {
        CW_Switch_On=0,
        CW_Enable_Voltage=1,
        CW_Quick_Stop=2,
        CW_Enable_Operation=3,
        CW_Operation_mode_specific0=4,
        CW_Operation_mode_specific1=5,
        CW_Operation_mode_specific2=6,
        CW_Fault_Reset=7,
        CW_Halt=8,
        CW_Operation_mode_specific3=9,
        // CW_Reserved1=10,
        CW_Manufacturer_specific0=11,
        CW_Manufacturer_specific1=12,
        CW_Manufacturer_specific2=13,
        CW_Manufacturer_specific3=14,
        CW_Manufacturer_specific4=15,
    };
    static bool setTransition(uint16_t &cw, const State402::InternalState &from, const State402::InternalState &to, State402::InternalState *next);
};

template<uint16_t MASK> class WordAccessor{
    uint16_t &word_;
public:
    WordAccessor(uint16_t &word) : word_(word) {}
    bool set(uint8_t bit){
        uint16_t val = MASK & (1<<bit);
        word_ |= val;
        return val;
   }
    bool reset(uint8_t bit){
        uint16_t val = MASK & (1<<bit);
        word_ &= ~val;
        return val;
    }
    bool get(uint8_t bit) const { return word_ & (1<<bit); }
    uint16_t get() const { return word_ & MASK; }
    WordAccessor & operator=(const uint16_t &val){
        uint16_t was = word_;
        word_ = (word_ & ~MASK) | (val & MASK);
        return *this;
    }
};

class Mode {
public:
    const uint16_t mode_id_;
    Mode(uint16_t id) : mode_id_(id) {}
    typedef WordAccessor<(1<<Command402::CW_Operation_mode_specific0)|(1<<Command402::CW_Operation_mode_specific1)|(1<<Command402::CW_Operation_mode_specific2)|(1<<Command402::CW_Operation_mode_specific3)> OpModeAccesser;
    virtual bool start() = 0;
    virtual bool read(const uint16_t &sw) = 0;
    virtual bool write(OpModeAccesser& cw) = 0;
    virtual bool setTarget(const double &val) { LOG("not implemented"); return false; }
    virtual ~Mode() {}
};


template<typename T> class ModeTargetHelper : public Mode {
    T target_;
    boost::atomic<bool> has_target_;

public:
    ModeTargetHelper(uint16_t mode) : Mode (mode) {}
    bool hasTarget() { return has_target_; }
    T getTarget() { return target_; }
    virtual bool setTarget(const double &val) {
        if(boost::math::isnan(val)){
            LOG("target command is not a number");
            return false;
        }

        using boost::numeric_cast;
        using boost::numeric::positive_overflow;
        using boost::numeric::negative_overflow;

        try
        {
            target_= numeric_cast<T>(val);
        }
        catch(negative_overflow&) {
            LOG("Command " << val << " does not fit into target, clamping to min limit");
            target_= std::numeric_limits<T>::min();
        }
        catch(positive_overflow&) {
            LOG("Command " << val << " does not fit into target, clamping to max limit");
            target_= std::numeric_limits<T>::max();
        }
        catch(...){
            LOG("Was not able to cast command " << val);
            return false;
        }

        has_target_ = true;
        return true;
    }
    virtual bool start() { has_target_ = false; return true; }
};

template<uint16_t ID, typename TYPE, uint16_t OBJ, uint8_t SUB, uint16_t CW_MASK> class ModeForwardHelper : public ModeTargetHelper<TYPE> {
    canopen::ObjectStorage::Entry<TYPE> target_entry_;
public:
    ModeForwardHelper(boost::shared_ptr<ObjectStorage> storage) : ModeTargetHelper<TYPE>(ID) {
        if(SUB) storage->entry(target_entry_, OBJ, SUB);
        else storage->entry(target_entry_, OBJ);
    }
    virtual bool read(const uint16_t &sw) { return true;}
    virtual bool write(Mode::OpModeAccesser& cw) {
        if(this->hasTarget()){
            cw = cw.get() | CW_MASK;
            target_entry_.set(this->getTarget());
            return true;
        }else{
            cw = cw.get() & ~CW_MASK;
            return false;
        }
    }
};

typedef ModeForwardHelper<MotorBase::Profiled_Velocity, int32_t, 0x60FF, 0, 0> ProfiledVelocityMode;
typedef ModeForwardHelper<MotorBase::Profiled_Torque, int16_t, 0x6071, 0, 0> ProfiledTorqueMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Position, int32_t, 0x607A, 0, 0> CyclicSynchronousPositionMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Velocity, int32_t, 0x60FF, 0, 0> CyclicSynchronousVelocityMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Torque, int16_t, 0x6071, 0, 0> CyclicSynchronousTorqueMode;
typedef ModeForwardHelper<MotorBase::Velocity, int16_t, 0x6042, 0, (1<<Command402::CW_Operation_mode_specific0)|(1<<Command402::CW_Operation_mode_specific1)|(1<<Command402::CW_Operation_mode_specific2)> VelocityMode;
typedef ModeForwardHelper<MotorBase::Interpolated_Position, int32_t, 0x60C1, 0x01, (1<<Command402::CW_Operation_mode_specific0)> InterpolatedPositionMode;

class ProfiledPositionMode : public ModeTargetHelper<int32_t> {
    canopen::ObjectStorage::Entry<int32_t> target_position_;
    int32_t last_target_;
    uint16_t sw_;
public:
    enum SW_masks {
        MASK_Reached = (1<<State402::SW_Target_reached),
        MASK_Acknowledged = (1<<State402::SW_Operation_mode_specific0),
        MASK_Error = (1<<State402::SW_Operation_mode_specific1),
    };
    enum CW_bits {
        CW_NewPoint =  Command402::CW_Operation_mode_specific0,
        CW_Immediate =  Command402::CW_Operation_mode_specific1,
        CW_Blending =  Command402::CW_Operation_mode_specific3,
    };
    ProfiledPositionMode(boost::shared_ptr<ObjectStorage> storage) : ModeTargetHelper(MotorBase::Profiled_Position) {
        storage->entry(target_position_, 0x607A);
    }
    virtual bool start() { sw_ = 0; last_target_= std::numeric_limits<double>::quiet_NaN(); return ModeTargetHelper::start(); }
    virtual bool read(const uint16_t &sw) { sw_ = sw; return (sw & MASK_Error) == 0; }
    virtual bool write(OpModeAccesser& cw) {
        cw.set(CW_Immediate);
        if(hasTarget()){
            int32_t target = getTarget();
            if((sw_ & MASK_Acknowledged) == 0 && target != last_target_){
                if(cw.get(CW_NewPoint)){
                    cw.reset(CW_NewPoint); // reset if needed
                }else{
                    target_position_.set(target);
                    cw.set(CW_NewPoint);
                    last_target_ = target;
                }
            } else if (sw_ & MASK_Acknowledged){
                cw.reset(CW_NewPoint);
            }
            return true;
        }
        return false;
    }
};

class HomingMode: public Mode{
protected:
    enum SW_bits {
        SW_Attained =  State402::SW_Operation_mode_specific0,
        SW_Error =  State402::SW_Operation_mode_specific1,
    };
    enum CW_bits {
        CW_StartHoming =  Command402::CW_Operation_mode_specific0,
    };
public:
    HomingMode() : Mode(MotorBase::Homing) {}
    virtual bool executeHoming(canopen::LayerStatus &status) = 0;
};

class DefaultHomingMode: public HomingMode{
    canopen::ObjectStorage::Entry<int8_t> homing_method_;
    boost::atomic<bool> execute_;

    boost::mutex mutex_;
    boost::condition_variable cond_;
    uint16_t status_;

    enum SW_masks {
        MASK_Reached = (1<<State402::SW_Target_reached),
        MASK_Attained = (1<<SW_Attained),
        MASK_Error = (1<<SW_Error),
    };
    bool error(canopen::LayerStatus &status, const std::string& msg) { execute_= false; status.error(msg); return false; }
public:
    DefaultHomingMode(boost::shared_ptr<ObjectStorage> storage) {
        storage->entry(homing_method_, 0x6098);
    }
    virtual bool start();
    virtual bool read(const uint16_t &sw);
    virtual bool write(OpModeAccesser& cw);

    virtual bool executeHoming(canopen::LayerStatus &status);
};

class Motor402 : public MotorBase
{
public:

    Motor402(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings)
    : MotorBase(name), status_word_(0),control_word_(0),
      switching_state_(State402::InternalState(settings.get_optional<unsigned int>("switching_state", static_cast<unsigned int>(State402::Operation_Enable)))),
      monitor_mode_(settings.get_optional<bool>("monitor_mode", true))
    {
        storage->entry(status_word_entry_, 0x6041);
        storage->entry(control_word_entry_, 0x6040);
        storage->entry(op_mode_display_, 0x6061);
        storage->entry(op_mode_, 0x6060);
        try{
            storage->entry(supported_drive_modes_, 0x6502);
        }
        catch(...){
        }
    }

    virtual bool setTarget(double val);
    virtual bool enterModeAndWait(uint16_t mode);
    virtual bool isModeSupported(uint16_t mode);
    virtual uint16_t getMode();

    template<typename T> bool registerMode(uint16_t mode) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&Motor402::createAndRegister<T>, this, mode))).second;
    }
    template<typename T, typename T1> bool registerMode(uint16_t mode, const T1& t1) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&Motor402::createAndRegister<T,T1>, this, mode, t1))).second;
    }
    template<typename T, typename T1, typename T2> bool registerMode(uint16_t mode, const T1& t1, const T2& t2) {
        return mode_allocators_.insert(std::make_pair(mode, boost::bind(&Motor402::createAndRegister<T,T1,T2>, this, mode, t1, t2))).second;
    }

    virtual void registerDefaultModes(boost::shared_ptr<ObjectStorage> storage){
        registerMode<ProfiledPositionMode> (MotorBase::Profiled_Position, storage);
        registerMode<VelocityMode> (MotorBase::Velocity, storage);
        registerMode<ProfiledVelocityMode> (MotorBase::Profiled_Velocity, storage);
        registerMode<ProfiledTorqueMode> (MotorBase::Profiled_Torque, storage);
        registerMode<DefaultHomingMode> (MotorBase::Homing, storage);
        registerMode<InterpolatedPositionMode> (MotorBase::Interpolated_Position, storage);
        registerMode<CyclicSynchronousPositionMode> (MotorBase::Cyclic_Synchronous_Position, storage);
        registerMode<CyclicSynchronousVelocityMode> (MotorBase::Cyclic_Synchronous_Velocity, storage);
        registerMode<CyclicSynchronousTorqueMode> (MotorBase::Cyclic_Synchronous_Torque, storage);
    }

    class Allocator : public MotorBase::Allocator{
    public:
        virtual boost::shared_ptr<MotorBase> allocate(const std::string &name, boost::shared_ptr<ObjectStorage> storage, const canopen::Settings &settings);
    };
protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleDiag(LayerReport &report);
    virtual void handleInit(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);

private:
    template<typename T> void createAndRegister0(uint16_t mode){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T()));
    }
    template<typename T, typename T1> void createAndRegister(uint16_t mode, const T1& t1){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T(t1)));
    }
    template<typename T, typename T1, typename T2> void createAndRegister(uint16_t mode, const T1& t1, const T2& t2){
        if(isModeSupportedByDevice(mode)) registerMode(mode, boost::shared_ptr<Mode>(new T(t1,t2)));
    }

    virtual bool isModeSupportedByDevice(uint16_t mode);
    void registerMode(uint16_t id, const boost::shared_ptr<Mode> &m);

    boost::shared_ptr<Mode> allocMode(uint16_t mode);

    bool readState(LayerStatus &status, const LayerState &current_state);
    bool switchMode(LayerStatus &status, uint16_t mode);
    bool switchState(LayerStatus &status, const State402::InternalState &target);

    boost::atomic<uint16_t> status_word_;
    uint16_t control_word_;
    boost::mutex cw_mutex_;
    boost::atomic<bool> start_fault_reset_;
    boost::atomic<State402::InternalState> target_state_;


    State402 state_handler_;

    boost::mutex map_mutex_;
    boost::unordered_map<uint16_t, boost::shared_ptr<Mode> > modes_;
    boost::unordered_map<uint16_t, boost::function<void()> > mode_allocators_;

    boost::shared_ptr<Mode> selected_mode_;
    uint16_t mode_id_;
    boost::condition_variable mode_cond_;
    boost::mutex mode_mutex_;
    const State402::InternalState switching_state_;
    const bool monitor_mode_;

    canopen::ObjectStorage::Entry<uint16_t>  status_word_entry_;
    canopen::ObjectStorage::Entry<uint16_t >  control_word_entry_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_display_;
    canopen::ObjectStorage::Entry<int8_t>  op_mode_;
    canopen::ObjectStorage::Entry<uint32_t>  supported_drive_modes_;
};

}

#endif
