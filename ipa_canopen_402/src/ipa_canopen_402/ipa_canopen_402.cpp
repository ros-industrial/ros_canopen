#include <ipa_canopen_402/ipa_canopen_402.h>

using namespace ipa_canopen;

//Node_402::Motor(const boost::shared_ptr<ipa_canopen::ObjectStorage> node_storage)
//: node_storage_(node_storage)
//{
//    configureEntries();
//}

const Node_402::State& Node_402::getState()
{

    return state_;
}

const int8_t Node_402::getMode()
{
    int8_t operation_mode = op_mode_display.get();

    return operation_mode;
}

const int32_t Node_402::getActualVel()
{
    int32_t ac_vel = actual_vel.get();
    return ac_vel;
}

const int32_t Node_402::getActualPos()
{
    int32_t ac_pos = actual_pos.get();
    return ac_pos;
}

const int32_t Node_402::getActualInternalPos()
{
    int32_t internal_pos = actual_internal_pos.get();
    return internal_pos;
}

void Node_402::setTargetVel(int32_t target_vel)
{
    target_velocity.set(target_vel);
}

void Node_402::setTargetPos(int32_t target_pos)
{
    int32_t oldpos = target_position.get_once();
    target_position.set(target_pos);
    if(oldpos != target_pos) operate();
}

bool Node_402::enterMode(const OperationMode &op_mode_var)
{
    op_mode.set(op_mode_var);

    if(getMode()==op_mode_var)
        return true;
    else
        return false;
}

void Node_402::configureEntries()
{
    getStorage()->entry(status_word, 0x6041);
    getStorage()->entry(control_word, 0x6040);

    getStorage()->entry(op_mode,0x6060);
    getStorage()->entry(op_mode_display,0x6061);

    getStorage()->entry(actual_vel,0x606C);
    getStorage()->entry(target_velocity,0x60FF);
    getStorage()->entry(profile_velocity,0x6081);

    getStorage()->entry(target_position,0x607A);
    getStorage()->entry(actual_pos,0x6064);
    getStorage()->entry(actual_internal_pos,0x6063);
}

bool Node_402::turnOn()
{
    control_word.set(0x06);
    control_word.set(0x07);
    control_word.set(0x0f);

    return true;
}

bool Node_402::operate()
{
    if(getMode()==Profiled_Position)
    {
        profile_velocity.set(1000000);
        control_word.set(0x1f);
        control_word.set(0x0f);
    }
    return true;
}

bool Node_402::turnOff()
{
    control_word.set(0x6);

    return true;
}

bool Node_402::init()
{
    
    enterMode(Profiled_Position);
    
    if(Node_402::turnOn()){
        operate();
        return true;
    }else
        return false;
}



void Node_402::switchState(const Node::State &s){
    switch(s){
        case Operational:
            Node_402::init();
            break;
        case BootUp:
        case PreOperational:
        case Stopped:
        default:
            //error
            ;
    }
}

