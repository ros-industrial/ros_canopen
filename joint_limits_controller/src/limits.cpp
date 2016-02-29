#include <joint_limits_controller/limited_joint_handle.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

void LimitedJointHandle::Limits::read(boost::shared_ptr<const urdf::Joint> joint){
    if(joint){
        if(joint_limits_interface::getJointLimits(joint, joint_limits)){
            limits_flags |= JointLimitsConfigured;
        }
        if(joint_limits_interface::getSoftJointLimits(joint, soft_limits)){
            limits_flags |= SoftLimitsConfigured;
        }
    }
}
void LimitedJointHandle::Limits::read(const std::string& name, ros::NodeHandle& nh, bool parse_soft_limits){
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

bool LimitedJointHandle::Limits::getAccelerationLimit(double &limit,const ros::Duration& period) const{
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

bool LimitedJointHandle::Limits::getVelocityLimit(double &limit,const ros::Duration& period) const{
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

bool LimitedJointHandle::Limits::valid() const {
    return true;
}

void LimitedJointHandle::Limits::merge(const Limits &other){
    *this = other; //TODO: proper merge
}

std::pair<double,double> LimitedJointHandle::Limits::getVelocitySoftBounds(double pos) const {
    std::pair<double,double> vel_soft_bounds = getSoftBounds(pos, soft_limits.k_position, soft_limits.min_position, soft_limits.max_position);


    if(hasVelocityLimits()){
        if(vel_soft_bounds.first < -joint_limits.max_velocity) vel_soft_bounds.first = -joint_limits.max_velocity;
        if(vel_soft_bounds.second > joint_limits.max_velocity) vel_soft_bounds.second = joint_limits.max_velocity;
    }

    return vel_soft_bounds;
}

bool LimitedJointHandle::Limits::hasPositionLimits() const {
    return (limits_flags & PositionLimitsConfigured) && joint_limits.has_position_limits && !joint_limits.angle_wraparound;
}

bool LimitedJointHandle::Limits::hasVelocityLimits() const {
    return (limits_flags & VelocityLimitsConfigured) && joint_limits.has_velocity_limits;
}

bool LimitedJointHandle::Limits::hasAccelerationLimits() const {
    return (limits_flags & AccelerationLimitsConfigured) && joint_limits.has_acceleration_limits;
}

bool LimitedJointHandle::Limits::hasJerkLimits() const {
    return (limits_flags & JerkLimitsConfigured) && joint_limits.has_jerk_limits;
}

bool LimitedJointHandle::Limits::hasEffortLimits() const {
    return (limits_flags & EffortLimitsConfigured) && joint_limits.has_effort_limits;
}

bool LimitedJointHandle::Limits::hasSoftLimits() const {
    return (limits_flags & SoftLimitsConfigured) && has_soft_limits;
}

double LimitedJointHandle::Limits::limitPosititon(double pos) const {
    if(hasPositionLimits()) return limitBounds(pos, joint_limits.min_position, joint_limits.max_position);
    else return pos;
}

double LimitedJointHandle::Limits::limitVelocity(double vel) const {
    if(hasVelocityLimits()) return limitBounds(vel, -joint_limits.max_velocity, joint_limits.max_velocity);
    else return vel;
}
double LimitedJointHandle::Limits::limitEffort(double eff) const {
    if(hasEffortLimits()) return limitBounds(eff, -joint_limits.max_effort, joint_limits.max_effort);
    else return eff;
}

double LimitedJointHandle::Limits::stopOnPositionLimit(double cmd, double current_pos) const {
    if(hasPositionLimits()){
        if((cmd > 0 && current_pos > joint_limits.max_position) ||
            (cmd < 0 && current_pos < joint_limits.min_position)){
            cmd = 0;
        }
    }
    return cmd;
}
