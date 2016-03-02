#ifndef CANOPEN_MOTOR_NODE_JOINT_LIMITER_H_
#define CANOPEN_MOTOR_NODE_JOINT_LIMITER_H_

#include <joint_limits_interface/joint_limits.h>
#include <urdf_model/joint.h>
#include <ros/node_handle.h>

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

class JointLimiter {
public:
    static double limitBounds(double value, double min, double max);

    static std::pair<double,double> getSoftBounds(double value, double k, double lower, double upper);

    class Limits {
        void read(boost::shared_ptr<const urdf::Joint> joint);
        void read(const std::string& joint_name, const ros::NodeHandle& nh, bool parse_soft_limits);
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
        Limits(const std::string& joint_name, const ros::NodeHandle& nh, bool parse_soft_limits) : limits_flags(0), has_soft_limits(false) { read(joint_name, nh, parse_soft_limits); }

        Limits(const Limits& other) { *this = other; }
        Limits(const Limits& base, const Limits& other) { *this = base; merge(other); }

        void merge(const Limits &other);

        void merge(const std::string& joint_name, const ros::NodeHandle& nh, bool parse_soft_limits){
            Limits l(joint_name, nh, parse_soft_limits);
            merge(l);
        }
        void merge(boost::shared_ptr<const urdf::Joint> joint){
            Limits l(joint);
            merge(l);
        }

        void apply(const Limits &other);

        void apply(const std::string& joint_name, const ros::NodeHandle& nh, bool parse_soft_limits){
            Limits l(joint_name, nh, parse_soft_limits);
            apply(l);
        }
        void apply(boost::shared_ptr<const urdf::Joint> joint){
            Limits l(joint);
            apply(l);
        }

        bool hasPositionLimits() const;
        bool hasVelocityLimits() const;
        bool hasAccelerationLimits() const;
        bool hasJerkLimits() const;
        bool hasEffortLimits() const;
        bool hasSoftLimits() const;

        void setPositionLimits(double min_position, double max_position);
        void setVelocityLimits(double max_velocity);
        void setAccelerationLimits(double max_acceleration);
        void setJerkLimits(double max_jerk);
        void setEffortLimits(double max_effort);
        void setSoftLimits(double k_position, double min_position, double max_position, double k_velocity);

        bool getVelocityLimit(double &limit, const double& period) const;
        bool getAccelerationLimit(double &limit, const double& period) const;

        std::pair<double,double> getVelocitySoftBounds(double pos) const;

        double limitPosition(double pos) const;
        double limitVelocity(double vel) const;
        double limitEffort(double eff) const;

        double stopOnPositionLimit(double cmd, double current_pos) const;

        bool valid() const;
    };
    virtual void enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) = 0;

    virtual void recover() {
        last_command_.reset();
    }
    ~JointLimiter() {}

protected:
    DataStore<double> last_command_;
};

class PositionJointLimiter : public JointLimiter {
public:
    virtual void enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd);
};

class VelocityJointLimiter : public JointLimiter {
public:
    virtual void enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd);
};

class EffortJointLimiter : public JointLimiter {
public:
    virtual void enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd);
};

#endif
