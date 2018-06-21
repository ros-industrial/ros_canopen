#include <canopen_402/motor.h>
#include <boost/thread/reverse_lock.hpp>

namespace canopen
{

State402::InternalState State402::getState(){
    boost::mutex::scoped_lock lock(mutex_);
    return state_;
}

State402::InternalState State402::read(uint16_t sw) {
    static const uint16_t r = (1 << SW_Ready_To_Switch_On);
    static const uint16_t s = (1 << SW_Switched_On);
    static const uint16_t o = (1 << SW_Operation_enabled);
    static const uint16_t f = (1 << SW_Fault);
    static const uint16_t q = (1 << SW_Quick_stop);
    static const uint16_t d = (1 << SW_Switch_on_disabled);

    InternalState new_state = Unknown;

    uint16_t state = sw & ( d | q | f | o | s | r );
    switch ( state )
    {
    //   ( d | q | f | o | s | r ):
    case ( 0 | 0 | 0 | 0 | 0 | 0 ):
    case ( 0 | q | 0 | 0 | 0 | 0 ):
        new_state = Not_Ready_To_Switch_On;
        break;

    case ( d | 0 | 0 | 0 | 0 | 0 ):
    case ( d | q | 0 | 0 | 0 | 0 ):
        new_state =  Switch_On_Disabled;
        break;

    case ( 0 | q | 0 | 0 | 0 | r ):
        new_state =  Ready_To_Switch_On;
        break;

    case ( 0 | q | 0 | 0 | s | r ):
        new_state =  Switched_On;
        break;

    case ( 0 | q | 0 | o | s | r ):
        new_state =  Operation_Enable;
        break;

    case ( 0 | 0 | 0 | o | s | r ):
        new_state =  Quick_Stop_Active;
        break;

    case ( 0 | 0 | f | o | s | r ):
    case ( 0 | q | f | o | s | r ):
        new_state =  Fault_Reaction_Active;
        break;

    case ( 0 | 0 | f | 0 | 0 | 0 ):
    case ( 0 | q | f | 0 | 0 | 0 ):
        new_state =  Fault;
        break;

    default:
        LOG("Motor is currently in an unknown state: " << std::hex <<  state << std::dec);
    }
    boost::mutex::scoped_lock lock(mutex_);
    if(new_state != state_){
        state_ = new_state;
        cond_.notify_all();
    }
    return state_;
}
bool State402::waitForNewState(const time_point &abstime, State402::InternalState &state){
    boost::mutex::scoped_lock lock(mutex_);
    while(state_ == state && cond_.wait_until(lock, abstime) == boost::cv_status::no_timeout) {}
    bool res = state != state_;
    state = state_;
    return res;
}

const Command402::TransitionTable Command402::transitions_;

Command402::TransitionTable::TransitionTable(){
    typedef State402 s;

    transitions_.reserve(32);

    Op disable_voltage(0,(1<<CW_Fault_Reset) | (1<<CW_Enable_Voltage));
    /* 7*/ add(s::Ready_To_Switch_On, s::Switch_On_Disabled, disable_voltage);
    /* 9*/ add(s::Operation_Enable, s::Switch_On_Disabled, disable_voltage);
    /*10*/ add(s::Switched_On, s::Switch_On_Disabled, disable_voltage);
    /*12*/ add(s::Quick_Stop_Active, s::Switch_On_Disabled, disable_voltage);

    Op automatic(0,0);
    /* 0*/ add(s::Start, s::Not_Ready_To_Switch_On, automatic);
    /* 1*/ add(s::Not_Ready_To_Switch_On, s::Switch_On_Disabled, automatic);
    /*14*/ add(s::Fault_Reaction_Active, s::Fault, automatic);

    Op shutdown((1<<CW_Quick_Stop) | (1<<CW_Enable_Voltage), (1<<CW_Fault_Reset) | (1<<CW_Switch_On));
    /* 2*/ add(s::Switch_On_Disabled, s::Ready_To_Switch_On, shutdown);
    /* 6*/ add(s::Switched_On, s::Ready_To_Switch_On, shutdown);
    /* 8*/ add(s::Operation_Enable, s::Ready_To_Switch_On, shutdown);

    Op switch_on((1<<CW_Quick_Stop) | (1<<CW_Enable_Voltage) | (1<<CW_Switch_On), (1<<CW_Fault_Reset) | (1<<CW_Enable_Operation));
    /* 3*/ add(s::Ready_To_Switch_On, s::Switched_On, switch_on);
    /* 5*/ add(s::Operation_Enable, s::Switched_On, switch_on);

    Op enable_operation((1<<CW_Quick_Stop) | (1<<CW_Enable_Voltage) | (1<<CW_Switch_On) | (1<<CW_Enable_Operation), (1<<CW_Fault_Reset));
    /* 4*/ add(s::Switched_On, s::Operation_Enable, enable_operation);
    /*16*/ add(s::Quick_Stop_Active, s::Operation_Enable, enable_operation);

    Op quickstop((1<<CW_Enable_Voltage), (1<<CW_Fault_Reset) | (1<<CW_Quick_Stop));
    /* 7*/ add(s::Ready_To_Switch_On, s::Quick_Stop_Active, quickstop); // transit to Switch_On_Disabled
    /*10*/ add(s::Switched_On, s::Quick_Stop_Active, quickstop); // transit to Switch_On_Disabled
    /*11*/ add(s::Operation_Enable, s::Quick_Stop_Active, quickstop);

    // fault reset
    /*15*/ add(s::Fault, s::Switch_On_Disabled, Op((1<<CW_Fault_Reset), 0));
}
State402::InternalState Command402::nextStateForEnabling(State402::InternalState state){
    switch(state){
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
}

bool Command402::setTransition(uint16_t &cw, const State402::InternalState &from, const State402::InternalState &to, State402::InternalState *next){
    try{
        if(from != to){
            State402::InternalState hop = to;
            if(next){
                if(to == State402::Operation_Enable) hop = nextStateForEnabling(from);
                *next = hop;
            }
            transitions_.get(from, hop)(cw);
        }
        return true;
    }
    catch(...){
        LOG("illegal tranistion " << from << " -> " << to);
    }
    return false;
}

template<uint16_t mask, uint16_t not_equal> struct masked_status_not_equal {
    uint16_t &status_;
    masked_status_not_equal(uint16_t &status) : status_(status) {}
    bool operator()() const { return (status_ & mask) != not_equal; }
};
bool DefaultHomingMode::start() {
    execute_ = false;
    return read(0);
}
bool DefaultHomingMode::read(const uint16_t &sw) {
    boost::mutex::scoped_lock lock(mutex_);
    uint16_t old = status_;
    status_ = sw & (MASK_Reached | MASK_Attained | MASK_Error);
    if(old != status_){
        cond_.notify_all();
    }
    return true;
}
bool DefaultHomingMode::write(Mode::OpModeAccesser& cw) {
    cw = 0;
    if(execute_){
        cw.set(CW_StartHoming);
        return true;
    }
    return false;
}

bool DefaultHomingMode::executeHoming(canopen::LayerStatus &status) {
    if(!homing_method_.valid()){
        return error(status, "homing method entry is not valid");
    }

    if(homing_method_.get_cached() == 0){
        return true;
    }

    time_point prepare_time = get_abs_time(boost::chrono::seconds(1));
    // ensure homing is not running
    boost::mutex::scoped_lock lock(mutex_);
    if(!cond_.wait_until(lock, prepare_time, masked_status_not_equal<MASK_Error | MASK_Reached, 0> (status_))){
        return error(status, "could not prepare homing");
    }
    if(status_ & MASK_Error){
        return error(status, "homing error before start");
    }

    execute_ = true;

    // ensure start
    if(!cond_.wait_until(lock, prepare_time, masked_status_not_equal<MASK_Error | MASK_Attained | MASK_Reached, MASK_Reached> (status_))){
        return error(status, "homing did not start");
    }
    if(status_ & MASK_Error){
        return error(status, "homing error at start");
    }

    time_point finish_time = get_abs_time(boost::chrono::seconds(10)); //

    // wait for attained
    if(!cond_.wait_until(lock, finish_time, masked_status_not_equal<MASK_Error | MASK_Attained, 0> (status_))){
        return error(status, "homing not attained");
    }
    if(status_ & MASK_Error){
        return error(status, "homing error during process");
    }

    // wait for motion stop
    if(!cond_.wait_until(lock, finish_time, masked_status_not_equal<MASK_Error | MASK_Reached, 0> (status_))){
        return error(status, "homing did not stop");
    }
    if(status_ & MASK_Error){
        return error(status, "homing error during stop");
    }

    if((status_ & MASK_Reached) && (status_ & MASK_Attained)){
        execute_ = false;
        return true;
    }

    return error(status, "something went wrong while homing");
}

bool Motor402::setTarget(double val){
    if(state_handler_.getState() == State402::Operation_Enable){
        boost::mutex::scoped_lock lock(mode_mutex_);
        return selected_mode_ && selected_mode_->setTarget(val);
    }
    return false;
}
bool Motor402::isModeSupported(uint16_t mode) { return mode != MotorBase::Homing && allocMode(mode); }

bool Motor402::enterModeAndWait(uint16_t mode) {
    LayerStatus s;
    bool okay = mode != MotorBase::Homing && switchMode(s, mode);
    if(!s.bounded<LayerStatus::Ok>()){
        LOG("Could not switch to mode " << mode << ", reason: " << s.reason());
    }
    return okay;
}

uint16_t Motor402::getMode() {
    boost::mutex::scoped_lock lock(mode_mutex_);
    return selected_mode_ ? selected_mode_->mode_id_ :  MotorBase::No_Mode;
}

bool Motor402::isModeSupportedByDevice(uint16_t mode){
    if(!supported_drive_modes_.valid()) {
        BOOST_THROW_EXCEPTION( std::runtime_error("Supported drive modes (object 6502) is not valid"));
    }
    return mode > 0 && mode <= 32 && (supported_drive_modes_.get_cached() & (1<<(mode-1)));
}
void Motor402::registerMode(uint16_t id, const ModeSharedPtr &m){
    boost::mutex::scoped_lock map_lock(map_mutex_);
    if(m && m->mode_id_ == id) modes_.insert(std::make_pair(id, m));
}

ModeSharedPtr Motor402::allocMode(uint16_t mode){
    ModeSharedPtr res;
    if(isModeSupportedByDevice(mode)){
        boost::mutex::scoped_lock map_lock(map_mutex_);
        boost::unordered_map<uint16_t, ModeSharedPtr >::iterator it = modes_.find(mode);
        if(it != modes_.end()){
            res = it->second;
        }
    }
    return res;
}

bool Motor402::switchMode(LayerStatus &status, uint16_t mode) {

    if(mode == MotorBase::No_Mode){
        boost::mutex::scoped_lock lock(mode_mutex_);
        selected_mode_.reset();
        try{ // try to set mode
            op_mode_.set(mode);
        }catch(...){}
        return true;
    }

    ModeSharedPtr next_mode = allocMode(mode);
    if(!next_mode){
        status.error("Mode is not supported.");
        return false;
    }

    if(!next_mode->start()){
        status.error("Could not start mode.");
        return false;
    }

    { // disable mode handler
        boost::mutex::scoped_lock lock(mode_mutex_);

        if(mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode){
            // nothing to do
            return true;
        }

        selected_mode_.reset();
    }

    if(!switchState(status, switching_state_)) return false;

    op_mode_.set(mode);

    bool okay = false;

    {  // wait for switch
        boost::mutex::scoped_lock lock(mode_mutex_);

        time_point abstime = get_abs_time(boost::chrono::seconds(5));
        if(monitor_mode_){
            while(mode_id_ != mode && mode_cond_.wait_until(lock, abstime) == boost::cv_status::no_timeout) {}
        }else{
            while(mode_id_ != mode && get_abs_time() < abstime){
                boost::reverse_lock<boost::mutex::scoped_lock> reverse(lock); // unlock inside loop
                op_mode_display_.get(); // poll
                boost::this_thread::sleep_for(boost::chrono::milliseconds(20)); // wait some time
            }
        }

        if(mode_id_ == mode){
            selected_mode_ = next_mode;
            okay = true;
        }else{
            status.error("Mode switch timed out.");
            op_mode_.set(mode_id_);
        }
    }

    if(!switchState(status, State402::Operation_Enable)) return false;

    return okay;

}

bool Motor402::switchState(LayerStatus &status, const State402::InternalState &target){
    time_point abstime = get_abs_time(state_switch_timeout_);
    State402::InternalState state = state_handler_.getState();
    target_state_ = target;
    while(state != target_state_){
        boost::mutex::scoped_lock lock(cw_mutex_);
        State402::InternalState next = State402::Unknown;
        if(!Command402::setTransition(control_word_ ,state, target_state_ , &next)){
            status.error("Could not set transition");
            return false;
        }
        lock.unlock();
        if(state != next && !state_handler_.waitForNewState(abstime, state)){
            status.error("Transition timeout");
            return false;
        }
    }
    return state == target;
}

bool Motor402::readState(LayerStatus &status, const LayerState &current_state){
    uint16_t old_sw, sw = status_word_entry_.get(); // TODO: added error handling
    old_sw = status_word_.exchange(sw);

    state_handler_.read(sw);

    boost::mutex::scoped_lock lock(mode_mutex_);
    uint16_t new_mode = monitor_mode_ ? op_mode_display_.get() : op_mode_display_.get_cached();
    if(selected_mode_ && selected_mode_->mode_id_ == new_mode){
        if(!selected_mode_->read(sw)){
            status.error("Mode handler has error");
        }
    }
    if(new_mode != mode_id_){
        mode_id_ = new_mode;
        mode_cond_.notify_all();
    }
    if(selected_mode_ && selected_mode_->mode_id_ != new_mode){
        status.warn("mode does not match");
    }
    if(sw & (1<<State402::SW_Internal_limit)){
        if(old_sw & (1<<State402::SW_Internal_limit) || current_state != Ready){
            status.warn("Internal limit active");
        }else{
            status.error("Internal limit active");
        }
    }

    return true;
}
void Motor402::handleRead(LayerStatus &status, const LayerState &current_state){
    if(current_state > Off){
        readState(status, current_state);
    }
}
void Motor402::handleWrite(LayerStatus &status, const LayerState &current_state){
    if(current_state > Off){
        boost::mutex::scoped_lock lock(cw_mutex_);
        control_word_ |= (1<<Command402::CW_Halt);
        if(state_handler_.getState() == State402::Operation_Enable){
            boost::mutex::scoped_lock lock(mode_mutex_);
            Mode::OpModeAccesser cwa(control_word_);
            bool okay = false;
            if(selected_mode_ && selected_mode_->mode_id_ == mode_id_){
                okay = selected_mode_->write(cwa);
            } else {
                cwa = 0;
            }
            if(okay) {
                control_word_ &= ~(1<<Command402::CW_Halt);
            }
        }
        if(start_fault_reset_.exchange(false)){
            control_word_entry_.set_cached(control_word_ & ~(1<<Command402::CW_Fault_Reset));
        }else{
            control_word_entry_.set_cached(control_word_);
        }
    }
}
void Motor402::handleDiag(LayerReport &report){
    uint16_t sw = status_word_;
    State402::InternalState state = state_handler_.getState();

    switch(state){
    case State402::Not_Ready_To_Switch_On:
    case State402::Switch_On_Disabled:
    case State402::Ready_To_Switch_On:
    case State402::Switched_On:
        report.warn("Motor operation is not enabled");
    case State402::Operation_Enable:
        break;

    case State402::Quick_Stop_Active:
        report.error("Quick stop is active");
        break;
    case State402::Fault:
    case State402::Fault_Reaction_Active:
        report.error("Motor has fault");
        break;
    case State402::Unknown:
        report.error("State is unknown");
        report.add("status_word", sw);
        break;
    }

    if(sw & (1<<State402::SW_Warning)){
        report.warn("Warning bit is set");
    }
    if(sw & (1<<State402::SW_Internal_limit)){
        report.error("Internal limit active");
    }
}
void Motor402::handleInit(LayerStatus &status){
    for(boost::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators_.begin(); it != mode_allocators_.end(); ++it){
        (it->second)();
    }

    if(!readState(status, Init)){
        status.error("Could not read motor state");
        return;
    }
    {
        boost::mutex::scoped_lock lock(cw_mutex_);
        control_word_ = 0;
        start_fault_reset_ = true;
    }
    if(!switchState(status, State402::Operation_Enable)){
        status.error("Could not enable motor");
        return;
    }

    ModeSharedPtr m = allocMode(MotorBase::Homing);
    if(!m){
        return; // homing not supported
    }

    HomingMode *homing = dynamic_cast<HomingMode*>(m.get());

    if(!homing){
        status.error("Homing mode has incorrect handler");
        return;
    }

    if(!switchMode(status, MotorBase::Homing)){
        status.error("Could not enter homing mode");
        return;
    }

    if(!homing->executeHoming(status)){
        status.error("Homing failed");
        return;
    }

    switchMode(status, No_Mode);
}
void Motor402::handleShutdown(LayerStatus &status){
    switchMode(status, MotorBase::No_Mode);
    switchState(status, State402::Switch_On_Disabled);
}
void Motor402::handleHalt(LayerStatus &status){
    State402::InternalState state = state_handler_.getState();
    boost::mutex::scoped_lock lock(cw_mutex_);

    // do not demand quickstop in case of fault
    if(state == State402::Fault_Reaction_Active || state == State402::Fault) return;

    if(state != State402::Operation_Enable){
        target_state_ = state;
    }else{
        target_state_ = State402::Quick_Stop_Active;
        if(!Command402::setTransition(control_word_ ,state, State402::Quick_Stop_Active, 0)){
            status.warn("Could not quick stop");
        }
    }
}
void Motor402::handleRecover(LayerStatus &status){
    start_fault_reset_ = true;
    {
        boost::mutex::scoped_lock lock(mode_mutex_);
        if(selected_mode_ && !selected_mode_->start()){
            status.error("Could not restart mode.");
            return;
        }
    }
    if(!switchState(status, State402::Operation_Enable)){
        status.error("Could not enable motor");
        return;
    }
}

} // namespace
