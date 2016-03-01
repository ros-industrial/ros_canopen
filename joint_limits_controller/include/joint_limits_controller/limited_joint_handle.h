#ifndef CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_
#define CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_

#include <joint_limits_controller/joint_limiter.h>
#include <hardware_interface/joint_command_interface.h>

class LimitedJointHandle: public hardware_interface::JointHandle {
public:
    typedef JointLimiter::Limits Limits;
    LimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd) : JointHandle(js,cmd) {}
    LimitedJointHandle(const hardware_interface::JointHandle &handle) : JointHandle(handle) {}
    virtual void enforceLimits(const ros::Duration& period, const JointLimiter::Limits &limits) = 0;
    virtual void enforceLimits(const ros::Duration& period) = 0;
    virtual Limits overlay(const Limits& limits) = 0;
    virtual void recover()  = 0;
};

template<typename T> class TypedLimitedJointHandle: public LimitedJointHandle {
    T limiter_;
    const Limits limits_;
public:
    TypedLimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd) : LimitedJointHandle(js,cmd) {}
    TypedLimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const JointLimiter::Limits &limits) : LimitedJointHandle(js,cmd), limits_(limits) {}
    TypedLimitedJointHandle(const hardware_interface::JointHandle &handle, const JointLimiter::Limits &limits) : LimitedJointHandle(handle), limits_(limits) {}

    virtual void enforceLimits(const ros::Duration& period, const JointLimiter::Limits &limits){
        double cmd = getCommand();
        limiter_.enforceLimits(period.toSec(), limits_, getPosition(), getVelocity(), getEffort(), cmd);
        setCommand(cmd);
    }
    virtual void enforceLimits(const ros::Duration& period) {
        enforceLimits(period, limits_);
    }
    virtual void recover()  { limiter_.recover(); }
    virtual Limits overlay(const Limits& limits){
        return Limits(limits_, limits);
    }
};

typedef TypedLimitedJointHandle<PositionJointLimiter> LimitedPositionJointHandle;
typedef TypedLimitedJointHandle<VelocityJointLimiter> LimitedVelocityJointHandle;
typedef TypedLimitedJointHandle<EffortJointLimiter> LimitedEffortJointHandle;

#endif
