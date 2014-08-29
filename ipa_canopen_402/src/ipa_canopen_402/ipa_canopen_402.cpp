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
    pos_ = actual_pos.get();
    return pos_;
}

const double Motor::getActualInternalPos()
{
    boost::timed_mutex::scoped_lock lock(mutex); // TODO: timed lock?
    pos_ = actual_internal_pos.get();
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
    node_storage_->entry(target_velocity,0x60FF);
    node_storage_->entry(profile_velocity,0x6081);

    node_storage_->entry(target_position,0x607A);
    node_storage_->entry(actual_pos,0x6064);
    node_storage_->entry(actual_internal_pos,0x6063);
}

bool Motor::turnOn()
{
    control_word.set(0x06);
    control_word.set(0x07);
    control_word.set(0x0f);

    return true;
}

bool Motor::operate()
{
    if(getMode()==Profiled_Position)
    {
        profile_velocity.set(1000000);
        control_word.set(0x1f);
        control_word.set(0x0f);
    }
}

bool Motor::turnOff()
{
    control_word.set(0x6);

    return true;
}

bool Motor::init()
{
    if(turnOn())
        return true;
    else
        return false;
}
