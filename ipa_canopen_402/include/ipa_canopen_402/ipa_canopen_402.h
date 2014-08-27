#ifndef H_IPA_CANOPEN_402
#define H_IPA_CANOPEN_402

#include <ipa_canopen_master/canopen.h>

namespace ipa_canopen_402{


ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  status_word;
ipa_canopen::ObjectStorage::Entry<ipa_canopen::ObjectStorage::DataType<0x006>::type >  control_word;
ipa_canopen::ObjectStorage::Entry<int8_t>  op_mode_display;
ipa_canopen::ObjectStorage::Entry<int8_t>  op_mode;

ipa_canopen::ObjectStorage::Entry<int32_t> actual_vel;
ipa_canopen::ObjectStorage::Entry<int32_t> target_velocity;

ipa_canopen::ObjectStorage::Entry<int32_t> actual_pos;
ipa_canopen::ObjectStorage::Entry<int32_t> target_position;


class Motor{
public:
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

    enum State{
        Start,
        Not_Ready_To_Switch_On,
        Switch_On_Disabled,
        Ready_To_Switch_On,
        Switched_On,
        Operation_Enable,
        Quick_Stop_Active,
        Fault_Reaction_Active
    };

    const boost::shared_ptr<ipa_canopen::ObjectStorage> node_storage_;

    Motor(const boost::shared_ptr<ipa_canopen::ObjectStorage> node_storage);

    const int8_t getMode();
    bool enterMode(const OperationMode &op_mode);

    const State& getState();
    void enterState(const State &s);

    const double getActualPos();
    void setTargetPos(int32_t pos);

    const double getActualVel();
    void setTargetVel(int32_t target_vel);

    bool init();
    void stop();
    void recover();
    void configureEntries();

    bool prepareToOperate();

private:
    template<typename T> void wait_for(const State &s, const T &timeout);

    boost::timed_mutex mutex;
    boost::mutex cond_mutex;
    boost::condition_variable cond;

    State state_;
    double pos_;
    double vel_;
    int8_t operation_mode_;

};

}  //  namespace ipa_canopen_402
#endif  // H_IPA_CANOPEN_402
