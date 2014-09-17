#ifndef H_IPA_CANOPEN_402
#define H_IPA_CANOPEN_402

#include <ipa_canopen_master/canopen.h>

namespace ipa_canopen
{
class Node_402 : public ipa_canopen::Layer
{
public:
  Node_402(boost::shared_ptr <ipa_canopen::Node> n, const std::string &name) : Layer(name), n_(n)
  {
    configureEntries();
  }

  enum StatusWord
  {
    SW_Ready_To_Switch_On,
    SW_Switched_On,
    SW_Operation_enabled,
    SW_Fault,
    SW_Voltage_enabled,
    SW_Quick_stop,
    SW_Switch_on_disabled,
    SW_Warning,
    SW_Manufacturer_specific0,
    SW_Remote,
    SW_Target_reached,
    SW_Internal_limit,
    SW_Operation_specific0,
    SW_Operation_specific1,
    SW_Manufacturer_specific1,
    SW_Manufacturer_specific2
  };

  enum ControlWord
  {
    CW_Switch_On,
    CW_Enable_Voltage,
    CW_Quick_Stop,
    CW_Enable_Operation,
    CW_Operation_mode_specific0,
    CW_Operation_mode_specific1,
    CW_Operation_mode_specific2,
    CW_Fault_Reset,
    CW_Halt,
    CW_Reserved0,
    CW_Reserved1,
    CW_Manufacturer_specific0,
    CW_Manufacturer_specific1,
    CW_Manufacturer_specific2,
    CW_Manufacturer_specific3,
    CW_Manufacturer_specific4,
  };

  enum OperationMode
  {
    No_Mode,
    Profiled_Position,
    Velocity,
    Profiled_Velocity,
    Profiled_Torque,
    Reserved,
    Homing,
    Interpolated_Position,
    Cyclic_Synchronous_Position,
  };

  enum State
  {
    Start,
    Not_Ready_To_Switch_On,
    Switch_On_Disabled,
    Ready_To_Switch_On,
    Switched_On,
    Operation_Enable,
    Quick_Stop_Active,
    Fault_Reaction_Active,
    Fault
  };

  const int8_t getMode();
  bool enterMode(const OperationMode &op_mode);

  const State& getState();
  void enterState(const State &s);


  virtual void read(LayerStatus &status);
  virtual void write(LayerStatus &status);

  virtual void report(LayerStatusExtended &status);

  virtual void init(LayerStatusExtended &status);
  virtual void shutdown(LayerStatus &status);

  virtual void halt(LayerStatus &status) {} // TODO
  virtual void recover(LayerStatusExtended &status);

  const double getActualPos();
  const double getActualInternalPos();

  const double getActualVel();
  const double getActualEff();

  void setTargetPos(const double &target_pos);
  void setTargetVel(const double &target_vel);
  void setTargetEff(const double &v);

  const double getTargetPos();
  const double getTargetVel();
  const double getTargetEff();

  bool turnOn();
  bool operate();
  bool turnOff();

  void configureEntries();

private:
  boost::shared_ptr <ipa_canopen::Node> n_;
  volatile bool running;
  State state_;
  State target_state_;

  ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  status_word;
  ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  control_word;
  ipa_canopen::ObjectStorage::Entry<int8_t>  op_mode_display;
  ipa_canopen::ObjectStorage::Entry<int8_t>  op_mode;

  ipa_canopen::ObjectStorage::Entry<int32_t> actual_vel;
  ipa_canopen::ObjectStorage::Entry<int32_t> target_velocity;
  ipa_canopen::ObjectStorage::Entry<uint32_t> profile_velocity;
  ipa_canopen::ObjectStorage::Entry<int32_t> actual_pos;
  ipa_canopen::ObjectStorage::Entry<int32_t> actual_internal_pos;
  ipa_canopen::ObjectStorage::Entry<int32_t> target_position;

  int32_t ac_vel_;
  double ac_eff_;

  int8_t operation_mode_;
  int8_t operation_mode_to_set_;
  bool check_mode;

  int32_t ac_pos_;
  int32_t internal_pos_;
  int32_t oldpos_;

  std::bitset<15> status_word_bitset;
  std::bitset<15> control_word_bitset;

  int32_t target_vel_;
  int32_t target_pos_;

  std::vector<int> control_word_buffer;

};
}  //  namespace ipa_canopen
#endif  // H_IPA_CANOPEN_402
