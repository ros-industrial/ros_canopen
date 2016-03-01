#ifndef CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_
#define CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_

#include <joint_limits_controller/joint_limiter.h>
#include <hardware_interface/joint_command_interface.h>

class LimitedJointHandle: public hardware_interface::JointHandle {
public:
    typedef JointLimiter::Limits Limits;
    LimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd) : JointHandle(js,cmd) {}
    LimitedJointHandle(const hardware_interface::JointHandle &handle) : JointHandle(handle) {}
    virtual void enforceLimits(const ros::Duration& period) = 0;
    virtual void recover()  = 0;
    virtual boost::shared_ptr<LimitedJointHandle> overlay(Limits &limits)  = 0;
};

template<typename T> class TypedLimitedJointHandle: public LimitedJointHandle {
    T limiter_;
public:
    TypedLimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const JointLimiter::Limits &limits) : LimitedJointHandle(js,cmd) , limiter_(limits) {}
    TypedLimitedJointHandle(const hardware_interface::JointHandle &handle, const JointLimiter::Limits &limits) : LimitedJointHandle(handle) , limiter_(limits) {}
    virtual void enforceLimits(const ros::Duration& period) {
        double cmd = getCommand();
        limiter_.enforceLimits(period, getPosition(), getVelocity(), getEffort(), cmd);
        setCommand(cmd);
    }
    virtual void recover()  { limiter_.recover(); }
    virtual boost::shared_ptr<LimitedJointHandle> overlay(Limits &limits) {
        Limits new_limits(limiter_.limits_);
        new_limits.merge(limits);
        return boost::shared_ptr<LimitedJointHandle>(new TypedLimitedJointHandle<T>(*this, new_limits));
    }
};

typedef TypedLimitedJointHandle<PositionJointLimiter> LimitedPositionJointHandle;
typedef TypedLimitedJointHandle<VelocityJointLimiter> LimitedVelocityJointHandle;
typedef TypedLimitedJointHandle<EffortJointLimiter> LimitedEffortJointHandle;

#endif
