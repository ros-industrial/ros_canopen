#ifndef CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_
#define CANOPEN_MOTOR_NODE_LIMITED_JOINT_HANDLE_H_

#include <joint_limits_interface/joint_limits.h>
#include <hardware_interface/joint_command_interface.h>
#include <urdf_model/joint.h>
#include <ros/node_handle.h>

class LimitedJointHandle : public hardware_interface::JointHandle{
public:
    static double limitBounds(double value, double min, double max);

    static std::pair<double,double> getSoftBounds(double value, double k, double lower, double upper);

    class Limits {
        void read(boost::shared_ptr<const urdf::Joint> joint);
        void read(const std::string& joint_name, ros::NodeHandle& nh, bool parse_soft_limits);
    public:
        enum {
            PositionLimitsConfigured = (1<<0),
            VelocityLimitsConfigured = (1<<1),
            AccelerationLimitsConfigured = (1<<2),
            JerkLimitsConfigured = (1<<3),
            EffortLimitsConfigured = (1<<4),
            JointLimitsConfigured = (1<<5)-1,
            SoftLimitsConfigured = (1<<5),
        };

        size_t limits_flags;
        bool has_soft_limits;
        joint_limits_interface::JointLimits joint_limits;
        joint_limits_interface::SoftJointLimits soft_limits;

        Limits() : limits_flags(0), has_soft_limits(false) {}
        Limits(boost::shared_ptr<const urdf::Joint> joint) : limits_flags(0), has_soft_limits(false) { read(joint); }
        Limits(const std::string& joint_name, ros::NodeHandle& nh, bool parse_soft_limits) : limits_flags(0), has_soft_limits(false) { read(joint_name, nh, parse_soft_limits); }

        Limits(const Limits& other) { *this = other; }

        void merge(const Limits &other);

        bool hasPositionLimits() const;
        bool hasVelocityLimits() const;
        bool hasAccelerationLimits() const;
        bool hasJerkLimits() const;
        bool hasEffortLimits() const;
        bool hasSoftLimits() const;

        bool getAccelerationLimit(double &limit,const ros::Duration& period) const;
        bool getVelocityLimit(double &limit,const ros::Duration& period) const;

        std::pair<double,double> getVelocitySoftBounds(double pos) const;


        double limitPosititon(double pos) const;
        double limitVelocity(double vel) const;
        double limitEffort(double eff) const;

        double stopOnPositionLimit(double cmd, double current_pos) const;

        bool valid() const;
    };
    LimitedJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const Limits &limits) : JointHandle(js, cmd), limits_(limits) {}
    virtual void enforceLimits(const ros::Duration& period) = 0;
    virtual void recover()  = 0;
    ~LimitedJointHandle() {}

protected:
    const Limits limits_;
};

template<typename T> class DataStore {
    bool has_data_;
    T data_;
public:
    DataStore(): has_data_(false) {}
    bool get(T &data){
        data = data_;
        return has_data_;
    }
    void set(const T &data){
        data_ = data;
        has_data_ = true;
    }
    double diff(T &data, double d){
        if(has_data_) return data - data_;
        else return d;
    }
    void reset() {
        has_data_ = false;
    }
};

class LimitedPositionJointHandle : public LimitedJointHandle {
    DataStore<double> last_command_;
public:
    LimitedPositionJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const Limits &limits) : LimitedJointHandle(js, cmd, limits){}
    virtual void enforceLimits(const ros::Duration& period);
    virtual void recover() {
        last_command_.reset();
    }
};

class LimitedVelocityJointHandle : public LimitedJointHandle {
    DataStore<double> last_command_;
public:
    LimitedVelocityJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const Limits &limits) : LimitedJointHandle(js, cmd, limits) {}
    virtual void enforceLimits(const ros::Duration& period);
    virtual void recover() {
        last_command_.reset();
    }
};

class LimitedEffortJointHandle : public LimitedJointHandle {
    DataStore<double> last_command_;
public:
    LimitedEffortJointHandle(const hardware_interface::JointStateHandle &js, double *cmd, const Limits &limits) : LimitedJointHandle(js, cmd, limits) {}
    virtual void enforceLimits(const ros::Duration& period);
    virtual void recover() {
        last_command_.reset();
    }
};

#endif
