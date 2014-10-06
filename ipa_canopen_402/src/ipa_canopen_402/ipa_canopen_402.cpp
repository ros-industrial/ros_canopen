#include <ipa_canopen_402/ipa_canopen_402.h>

using namespace ipa_canopen;

void Node_402::read(LayerStatus &status)
{
  std::bitset<16> sw_new(status_word.get());

  status_word_bitset = sw_new;

  switch((status_word_bitset & status_word_mask).to_ulong())
  {
  case 0b0000000:
  case 0b0100000: state_ = Not_Ready_To_Switch_On; break;
  case 0b1000000:
  case 0b1100000: state_ = Switch_On_Disabled; break;
  case 0b0100001: state_ = Ready_To_Switch_On; break;
  case 0b0100011: state_ = Switched_On; break;
  case 0b0100111: state_ = Operation_Enable; break;
  case 0b0000111: state_ = Quick_Stop_Active; break;
  case 0b0001111:
  case 0b0101111: state_ = Fault_Reaction_Active; break;
  case 0b0001000:
  case 0b0101000: state_ = Fault; break;
  default:
    LOG("Motor currently in an unknown state");
    //status.error();
  }


  operation_mode_ = (OperationMode) op_mode_display.get(); // TODO: check validity
  ac_vel_ = actual_vel.get();
  ac_pos_ = actual_pos.get();
  oldpos_ = ac_pos_;//target_position.get_cached();
  if(check_mode)
  {
    target_pos_ = ac_pos_;
    target_vel_ = 0;
    if(operation_mode_ == operation_mode_to_set_)
    {
      check_mode = false;
    }
    else
    {
      status.warn();
    }
  }

}

void Node_402::shutdown(LayerStatus &status)
{

}

void Node_402::report(LayerStatusExtended &status)
{

}

void Node_402::recover(LayerStatusExtended &status)
{
  control_word_bitset.reset();
  target_vel_ = 0;
  target_pos_ = actual_pos.get_cached();
  LOG("RECOVER");
}

const double Node_402::getTargetPos()
{
  return target_pos_;
}
const double Node_402::getTargetVel()
{
  return target_vel_;
}

void Node_402::write(LayerStatus &status)
{
  if (state_ != target_state_)
  {
    if (state_ == Fault)
    {
      control_word_bitset.set(CW_Fault_Reset);
    }
    else
      control_word_bitset.reset(CW_Fault_Reset);

    if (state_ == Not_Ready_To_Switch_On || state_ == Switch_On_Disabled)
    {
      // 0x06 Shutdown
      control_word_bitset.reset(CW_Switch_On);
      control_word_bitset.set(CW_Enable_Voltage);
      control_word_bitset.set(CW_Quick_Stop);
    }

    else if(state_ == Ready_To_Switch_On)
    {
      if (target_state_ == Switch_On_Disabled)
      {
        control_word_bitset.set(CW_Enable_Voltage);
      }
      else
      {
        control_word_bitset.set(CW_Enable_Voltage);
        control_word_bitset.set(CW_Switch_On);
        control_word_bitset.set(CW_Quick_Stop);
      }
    }

    else if (state_ == Switched_On)
    {
      if (target_state_ == Switch_On_Disabled)
      {
        control_word_bitset.reset(CW_Enable_Voltage);
      }
      else if (target_state_ == Ready_To_Switch_On)
      {
        // 0x06 Shutdown
        control_word_bitset.reset(CW_Switch_On);
        control_word_bitset.set(CW_Enable_Voltage);
        control_word_bitset.set(CW_Quick_Stop);
      }
      else
      {
        // 0x07 Enable Operation
        control_word_bitset.set(CW_Switch_On);
        control_word_bitset.set(CW_Enable_Voltage);
        control_word_bitset.set(CW_Enable_Operation);
        control_word_bitset.set(CW_Quick_Stop);
      }
    }
    else if (state_ == Operation_Enable)
    {
      if (target_state_ == Switch_On_Disabled)
      {
        control_word_bitset.reset(CW_Enable_Voltage);
      }
      else if (target_state_ == Ready_To_Switch_On)
      {
        // 0x06 Shutdown
        control_word_bitset.reset(CW_Switch_On);
        control_word_bitset.set(CW_Enable_Voltage);
        control_word_bitset.set(CW_Quick_Stop);
      }
      //Disable operation ?
    }
    status.warn();
  }
  else if(state_ == Operation_Enable)
  {
    if(operation_mode_ == Profiled_Position)
    {
      if(new_target_pos_)
      {
        control_word_bitset.reset(CW_Operation_mode_specific0);
        control_word_bitset.reset(CW_Operation_mode_specific1);
        control_word_bitset.reset(CW_Operation_mode_specific2);
        new_target_pos_ = false;
      }
      else if(oldpos_ != target_pos_)
      {
        profile_velocity.set(10000);
        target_position.set(target_pos_);
        new_target_pos_ = true;
        control_word_bitset.set(CW_Operation_mode_specific0);
        control_word_bitset.reset(CW_Operation_mode_specific1);
        control_word_bitset.reset(CW_Operation_mode_specific2);
      }
    }
    else if(operation_mode_ == Profiled_Velocity)
    {
      target_profiled_velocity.set(target_vel_);
      control_word_bitset.reset(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
    }
    else if(operation_mode_ == Interpolated_Position)
    {
      target_interpolated_position.set(target_pos_);
      control_word_bitset.set(CW_Operation_mode_specific0);
      control_word_bitset.reset(CW_Operation_mode_specific1);
      control_word_bitset.reset(CW_Operation_mode_specific2);
    }
    else if(operation_mode_ == Velocity)
    {
      target_velocity.set(target_vel_);
      control_word_bitset.set(CW_Operation_mode_specific0);
      control_word_bitset.set(CW_Operation_mode_specific1);
      control_word_bitset.set(CW_Operation_mode_specific2);
    }
  }
  int16_t cw_set = static_cast<int>(control_word_bitset.to_ulong());
  control_word.set(cw_set);
}

const Node_402::State& Node_402::getState()
{
  return state_;
}

const Node_402::OperationMode Node_402::getMode()
{
  return operation_mode_;
}

bool Node_402::isModeSupported(const OperationMode &op_mode)
{
  return supported_drive_modes.get_cached() & getModeMask(op_mode);
}
uint32_t Node_402::getModeMask(const OperationMode &op_mode)
{
  SupportedOperationMode sup_mode = static_cast<SupportedOperationMode>((op_mode)-1);

  switch(op_mode)
  {
  case Sup_Profiled_Position:
  case Sup_Velocity:
  case Sup_Profiled_Velocity:
  case Sup_Profiled_Torque:
  case Sup_Interpolated_Position:
  case Sup_Cyclic_Synchronous_Position:
  case Sup_Cyclic_Synchronous_Velocity:
  case Sup_Cyclic_Synchronous_Torque:
    return (1<<(op_mode-1));
  case Sup_Reserved:
  case Sup_Homing:
    return 0;
  }
  return 0;
}
bool Node_402::isModeMaskRunning(const uint32_t &mask)
{
  return mask & getModeMask(operation_mode_);
}

const double Node_402::getActualVel()
{
  return ac_vel_;
}

const double Node_402::getActualEff()
{
  return ac_eff_;
}

const double Node_402::getActualPos()
{
  return ac_pos_;
}

const double Node_402::getActualInternalPos()
{
  return internal_pos_;
}

void Node_402::setTargetVel(const double &target_vel)
{
  target_vel_ = target_vel;
}

void Node_402::setTargetPos(const double &target_pos)
{
  target_pos_ = target_pos;
}

bool Node_402::enterMode(const OperationMode &op_mode_var)
{
  op_mode.set(op_mode_var);

  operation_mode_to_set_ = op_mode_var;

  check_mode = true;

  return true;
}

void Node_402::configureEntries()
{
  n_->getStorage()->entry(status_word, 0x6041);
  n_->getStorage()->entry(control_word, 0x6040);

  n_->getStorage()->entry(op_mode,0x6060);
  n_->getStorage()->entry(op_mode_display,0x6061);
  n_->getStorage()->entry(supported_drive_modes,0x6502);

  n_->getStorage()->entry(actual_vel,0x606C);

  n_->getStorage()->entry(actual_pos,0x6064);
}

void Node_402::configureModeSpecificEntries()
{
  if(isModeSupported(Profiled_Position))
  {
    n_->getStorage()->entry(target_position,0x607A);
    n_->getStorage()->entry(profile_velocity,0x6081);
  }
  if(isModeSupported(Profiled_Velocity))
  {
    n_->getStorage()->entry(target_profiled_velocity,0x60FF);
  }
  if(isModeSupported(Interpolated_Position))
  {
    n_->getStorage()->entry(target_interpolated_position,0x60C1,0x01);
  }
  if(isModeSupported(Velocity))
  {
    n_->getStorage()->entry(target_velocity,0x6042);
  }
}

bool Node_402::turnOn()
{
  target_state_ = Operation_Enable;

  return true;
}

bool Node_402::turnOff()
{
  target_state_ = Switch_On_Disabled;
  return true;
}

void Node_402::init(LayerStatusExtended &s)
{
  Node_402::configureModeSpecificEntries();

  if(Node_402::turnOn())
  {
    running=true;
  }
  else
    s.error();
}
