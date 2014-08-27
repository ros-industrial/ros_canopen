#include <ipa_canopen_402/ipa_canopen_402.h>

using ipa_canopen_402::Motor;

Motor::Motor(const boost::shared_ptr<ipa_canopen::ObjectStorage> node_storage)
: node_storage_(node_storage)
{
    configureEntries();
}

const Motor::State& Motor::getState()
{
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    return state_;
}

const int8_t Motor::getMode()
{
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    operation_mode_ = op_mode_display.get();

    return operation_mode_;
}

const double Motor::getActualVel()
{
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    vel_ = actual_vel.get();

    return vel_;
}

const double Motor::getActualPos()
{
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    return pos_;
}

void Motor::setTargetVel(int32_t target_vel)
{
    target_velocity.set(target_vel);
}

void Motor::setTargetPos(int32_t target_pos)
{
    target_position.set(target_pos);
}

bool Motor::enterMode(const OperationMode &op_mode_var)
{
    op_mode.set(op_mode_var);

    if(getMode()==op_mode_var)
        return true;
    else
        return false;
}

void Motor::configureEntries()
{
    node_storage_->entry(status_word, 0x6041);
    node_storage_->entry(control_word, 0x6040);
    node_storage_->entry(op_mode,0x6060);
    node_storage_->entry(op_mode_display,0x6061);
    node_storage_->entry(actual_vel,0x606C);
    node_storage_->entry(target_velocity,0x60ff);
    node_storage_->entry(target_position,0x607a);
    node_storage_->entry(actual_pos,0x6064);
}

bool Motor::prepareToOperate()
{
    control_word.set(0x6);
    control_word.set(0x7);
    control_word.set(0xf);

    return true;
}

bool Motor::init()
{
    if(prepareToOperate())
        return true;
    else
        return false;
}
