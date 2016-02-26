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
        void read(const std::string& joint_name, ros::NodeHandle& nh, bool soft_limits);
    public:
        bool has_joint_limits;
        bool has_soft_limits;
        joint_limits_interface::JointLimits joint_limits;
        joint_limits_interface::SoftJointLimits soft_limits;

        Limits() : has_joint_limits(false), has_soft_limits(false) {}
        Limits(boost::shared_ptr<const urdf::Joint> joint) : has_joint_limits(false), has_soft_limits(false) { read(joint); }
        Limits(const std::string& joint_name, ros::NodeHandle& nh, bool soft_limits) : has_joint_limits(false), has_soft_limits(false) { read(joint_name, nh, soft_limits); }

        Limits(const Limits& other) { *this = other; }

        void merge(const Limits &other);

        bool getAccelerationLimit(double &limit,const ros::Duration& period) const;
        bool getVelocityLimit(double &limit,const ros::Duration& period) const;

        std::pair<double,double> getVelocitySoftBounds(double pos) const;

        bool hasPositionLimits() const;
        bool hasVelocityLimits() const;

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
