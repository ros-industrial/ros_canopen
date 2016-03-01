#include <joint_limits_controller/limited_joint_handle.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

void JointLimiter::Limits::read(boost::shared_ptr<const urdf::Joint> joint){
    if(joint){
        if(joint_limits_interface::getJointLimits(joint, joint_limits)){
            limits_flags |= JointLimitsConfigured;
        }
        if(joint_limits_interface::getSoftJointLimits(joint, soft_limits)){
            limits_flags |= SoftLimitsConfigured;
        }
    }
}
void JointLimiter::Limits::read(const std::string& name, const ros::NodeHandle& nh, bool parse_soft_limits){
    ros::NodeHandle limits_nh(nh, "joint_limits/" + name);
    if(joint_limits_interface::getJointLimits(name, nh, joint_limits)){
         if(limits_nh.hasParam("has_position_limits")) limits_flags |= PositionLimitsConfigured;
         if(limits_nh.hasParam("has_velocity_limits")) limits_flags |= VelocityLimitsConfigured;
         if(limits_nh.hasParam("has_acceleration_limits")) limits_flags |= AccelerationLimitsConfigured;
         if(limits_nh.hasParam("has_jerk_limits")) limits_flags |= JerkLimitsConfigured;
         if(limits_nh.hasParam("has_effort_limits")) limits_flags |= EffortLimitsConfigured;
    }

    if(parse_soft_limits && limits_nh.getParam("has_soft_limits", has_soft_limits)){
        if(has_soft_limits){
            if(!limits_nh.getParam("soft_lower_limit", soft_limits.min_position)){
                ROS_ERROR("soft_lower_limit not set");
                return;
            }
            if(!limits_nh.getParam("soft_upper_limit", soft_limits.max_position)){
                ROS_ERROR("soft_upper_limit not set");
                return;
            }
            if(!limits_nh.getParam("k_position", soft_limits.k_position)){
                ROS_ERROR("k_position not set");
                return;
            }
            if(!limits_nh.getParam("k_velocity", soft_limits.k_velocity)){
                ROS_ERROR("k_velocity not set");
                return;
            }
        }
        limits_flags |= SoftLimitsConfigured;

    }
}

bool JointLimiter::Limits::getAccelerationLimit(double &limit,const ros::Duration& period) const{
    if(hasJerkLimits()){
        limit = period.toSec() * joint_limits.max_jerk;
        if(joint_limits.has_acceleration_limits && limit > joint_limits.max_acceleration) limit = joint_limits.max_acceleration;
    }else if(hasAccelerationLimits()){
        limit = joint_limits.max_acceleration;
    }else{
        return false;
    }
    return true;
}

bool JointLimiter::Limits::getVelocityLimit(double &limit,const ros::Duration& period) const{
    double a;
    if(getAccelerationLimit(a, period)){
        limit = a*period.toSec();
        if(joint_limits.has_velocity_limits && limit > joint_limits.max_velocity) limit = joint_limits.max_velocity;
    } else if(hasVelocityLimits()){
        limit = joint_limits.max_velocity;
    }else{
        return false;
    }
    return true;
}

bool JointLimiter::Limits::valid() const {
    return true;
}

#define TRY_MERGE(flag, max_prop, has_prop, msg)                                        \
    do {                                                                                \
        if(other.limits_flags & flag){                                                  \
            bool ok = true;                                                             \
            if((limits_flags & flag ) && joint_limits.has_prop){                        \
                if(!other.joint_limits.has_prop){                                       \
                    ok = false;                                                         \
                    ROS_ERROR_STREAM("Cannot disable " << msg << " limits");            \
                }else if(other.joint_limits.max_prop > joint_limits.max_prop){          \
                    ok = false;                                                         \
                    ROS_ERROR_STREAM("new joint_limits.max_" << msg << " too high");    \
                }                                                                       \
            }                                                                           \
            if(ok){                                                                     \
                limits_flags |= flag;                                                   \
                joint_limits.max_prop = other.joint_limits.max_prop;                    \
                joint_limits.has_prop = other.joint_limits.has_prop;                    \
            }                                                                           \
        }                                                                               \
    }while(0);

void JointLimiter::Limits::merge(const Limits &other){

    if(other.limits_flags & PositionLimitsConfigured){
        bool ok = true;
        if(hasPositionLimits()){
            if(!other.hasPositionLimits()){
                ok = false;
                ROS_ERROR("Cannot disable position limits");
            }else if(other.joint_limits.min_position < joint_limits.min_position){
                ok = false;
                ROS_ERROR("new joint_limits.min_position too low");
            }else if(other.joint_limits.max_position > joint_limits.max_position){
                ok = false;
                ROS_ERROR("new joint_limits.max_position too high");
            }else if(other.joint_limits.angle_wraparound != joint_limits.angle_wraparound){
                ok = false;
                ROS_ERROR("Cannot overwrite position limits");
            }
        }
        if(ok){
            limits_flags |= PositionLimitsConfigured;
            joint_limits.min_position = other.joint_limits.min_position;
            joint_limits.max_position = other.joint_limits.max_position;
            joint_limits.has_position_limits = other.joint_limits.has_position_limits;
        }
    }

    TRY_MERGE(VelocityLimitsConfigured, max_velocity, has_velocity_limits, "velocity");
    TRY_MERGE(AccelerationLimitsConfigured, max_acceleration, has_acceleration_limits, "acceleration");
    TRY_MERGE(JerkLimitsConfigured, max_jerk, has_jerk_limits, "jerk");
    TRY_MERGE(EffortLimitsConfigured, max_effort, has_effort_limits, "effort");


    if(other.limits_flags & SoftLimitsConfigured){
        if(has_soft_limits){ // merge
            if(other.soft_limits.min_position < soft_limits.min_position){
                ROS_ERROR("new soft_limits.min_position too low");
            }else if(other.soft_limits.max_position > soft_limits.max_position){
                ROS_ERROR("new soft_limits.max_position too high");
            }else if(other.soft_limits.k_position > soft_limits.k_position){
                ROS_ERROR("new soft_limits.k_position too high");
            }else if(other.soft_limits.k_velocity > soft_limits.k_velocity){
                ROS_ERROR("new soft_limits.k_velocity too high");
            }else{
                limits_flags |= SoftLimitsConfigured;
                soft_limits = other.soft_limits;
                has_soft_limits = other.has_soft_limits;
            }
        }else{
            limits_flags |= SoftLimitsConfigured;
            soft_limits = other.soft_limits;
            has_soft_limits = other.has_soft_limits;
        }
    }
}

void JointLimiter::Limits::merge(const std::string& joint_name, const ros::NodeHandle& nh, bool parse_soft_limits){
    Limits l(joint_name, nh, parse_soft_limits);
    merge(l);
}

void JointLimiter::Limits::merge(boost::shared_ptr<const urdf::Joint> joint){
    Limits l(joint);
    merge(l);
}

std::pair<double,double> JointLimiter::Limits::getVelocitySoftBounds(double pos) const {
    std::pair<double,double> vel_soft_bounds = getSoftBounds(pos, soft_limits.k_position, soft_limits.min_position, soft_limits.max_position);


    if(hasVelocityLimits()){
        if(vel_soft_bounds.first < -joint_limits.max_velocity) vel_soft_bounds.first = -joint_limits.max_velocity;
        if(vel_soft_bounds.second > joint_limits.max_velocity) vel_soft_bounds.second = joint_limits.max_velocity;
    }

    return vel_soft_bounds;
}

bool JointLimiter::Limits::hasPositionLimits() const {
    return (limits_flags & PositionLimitsConfigured) && joint_limits.has_position_limits && !joint_limits.angle_wraparound;
}

bool JointLimiter::Limits::hasVelocityLimits() const {
    return (limits_flags & VelocityLimitsConfigured) && joint_limits.has_velocity_limits;
}

bool JointLimiter::Limits::hasAccelerationLimits() const {
    return (limits_flags & AccelerationLimitsConfigured) && joint_limits.has_acceleration_limits;
}

bool JointLimiter::Limits::hasJerkLimits() const {
    return (limits_flags & JerkLimitsConfigured) && joint_limits.has_jerk_limits;
}

bool JointLimiter::Limits::hasEffortLimits() const {
    return (limits_flags & EffortLimitsConfigured) && joint_limits.has_effort_limits;
}

bool JointLimiter::Limits::hasSoftLimits() const {
    return (limits_flags & SoftLimitsConfigured) && has_soft_limits;
}

double JointLimiter::Limits::limitPosititon(double pos) const {
    if(hasPositionLimits()) return limitBounds(pos, joint_limits.min_position, joint_limits.max_position);
    else return pos;
}

double JointLimiter::Limits::limitVelocity(double vel) const {
    if(hasVelocityLimits()) return limitBounds(vel, -joint_limits.max_velocity, joint_limits.max_velocity);
    else return vel;
}
double JointLimiter::Limits::limitEffort(double eff) const {
    if(hasEffortLimits()) return limitBounds(eff, -joint_limits.max_effort, joint_limits.max_effort);
    else return eff;
}

double JointLimiter::Limits::stopOnPositionLimit(double cmd, double current_pos) const {
    if(hasPositionLimits()){
        if((cmd > 0 && current_pos > joint_limits.max_position) ||
            (cmd < 0 && current_pos < joint_limits.min_position)){
            cmd = 0;
        }
    }
    return cmd;
}
