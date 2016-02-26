#include <joint_limits_controller/limited_joint_handle.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

void LimitedJointHandle::Limits::read(boost::shared_ptr<const urdf::Joint> joint){
    has_joint_limits = joint_limits_interface::getJointLimits(joint, joint_limits);
    has_soft_limits = joint_limits_interface::getSoftJointLimits(joint, soft_limits);
}
void LimitedJointHandle::Limits::read(const std::string& name, ros::NodeHandle& nh, bool soft_limits){
    if(!nh.param("joint_limits/has_joint_limits", true)){
        has_joint_limits = false;
        joint_limits.has_position_limits = false;
        joint_limits.has_velocity_limits = false;
        joint_limits.has_acceleration_limits = false;
        joint_limits.has_jerk_limits = false;
        joint_limits.has_effort_limits = false;
    }
    has_joint_limits = joint_limits_interface::getJointLimits(name, nh, joint_limits) || has_joint_limits;

    if(!nh.param("joint_limits/has_soft_limits", true)) has_soft_limits = false;
}

bool LimitedJointHandle::Limits::getAccelerationLimit(double &limit,const ros::Duration& period) const{
    if(!has_joint_limits) return false;

    if(joint_limits.has_jerk_limits){
        limit = period.toSec() * joint_limits.max_jerk;
        if(joint_limits.has_acceleration_limits && limit > joint_limits.max_acceleration) limit = joint_limits.max_acceleration;
    }else if(joint_limits.has_acceleration_limits){
        limit = joint_limits.max_acceleration;
    }else{
        return false;
    }
    return true;
}

bool LimitedJointHandle::Limits::getVelocityLimit(double &limit,const ros::Duration& period) const{
    if(!has_joint_limits) return false;

    double a;
    if(getAccelerationLimit(a, period)){
        limit = a*period.toSec();
        if(joint_limits.has_velocity_limits && limit > joint_limits.max_velocity) limit = joint_limits.max_velocity;
    } else if(joint_limits.has_velocity_limits){
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
    return has_joint_limits && joint_limits.has_position_limits && !joint_limits.angle_wraparound;
}

bool LimitedJointHandle::Limits::hasVelocityLimits() const {
    return has_joint_limits && joint_limits.has_velocity_limits;
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
    if(has_joint_limits && joint_limits.has_effort_limits) return limitBounds(eff, -joint_limits.max_effort, joint_limits.max_effort);
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
