#include <joint_limits_controller/limited_joint_handle.h>

#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

bool JointLimiter::Limits::operator==(const Limits &other) const{
    return (limits_flags == other.limits_flags) && //TODO: only test configured settings
           (has_soft_limits == other.has_soft_limits) &&
           (joint_limits.min_position == other.joint_limits.min_position) &&
           (joint_limits.max_position == other.joint_limits.max_position) &&
           (joint_limits.max_velocity == other.joint_limits.max_velocity) &&
           (joint_limits.max_acceleration == other.joint_limits.max_acceleration) &&
           (joint_limits.max_jerk == other.joint_limits.max_jerk) &&
           (joint_limits.max_effort == other.joint_limits.max_effort) &&
           (joint_limits.has_position_limits == other.joint_limits.has_position_limits) &&
           (joint_limits.has_velocity_limits == other.joint_limits.has_velocity_limits) &&
           (joint_limits.has_acceleration_limits == other.joint_limits.has_acceleration_limits) &&
           (joint_limits.has_jerk_limits == other.joint_limits.has_jerk_limits) &&
           (joint_limits.has_effort_limits == other.joint_limits.has_effort_limits) &&
           (joint_limits.angle_wraparound == other.joint_limits.angle_wraparound) &&
           (soft_limits.min_position == other.soft_limits.min_position) &&
           (soft_limits.max_position == other.soft_limits.max_position) &&
           (soft_limits.k_position == other.soft_limits.k_position) &&
           (soft_limits.k_velocity == other.soft_limits.k_velocity);
}

bool JointLimiter::Limits::parseURDF(ros::NodeHandle nh, urdf::Model &urdf, bool *has_urdf){
   std::string has_robot_description_fqn;
   bool has_robot_description = true;

   if(nh.searchParam("has_robot_description", has_robot_description_fqn)){
        ros::param::get(has_robot_description_fqn, has_robot_description);
    }
    if(has_robot_description){
        if(!urdf.initParam("robot_description")) return false;
        if(has_urdf) *has_urdf = true;
    }else if(has_urdf){
        *has_urdf = false;
    }
    return true;
}

void JointLimiter::Limits::read(boost::shared_ptr<const urdf::Joint> joint){
    if(joint){
        if(joint_limits_interface::getJointLimits(joint, joint_limits)){
            limits_flags |= PositionLimitsConfigured;
            limits_flags |= VelocityLimitsConfigured;
            limits_flags |= EffortLimitsConfigured;
        }
        if(joint_limits_interface::getSoftJointLimits(joint, soft_limits)){
            limits_flags |= SoftLimitsConfigured;
            has_soft_limits = true;
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
    if(hasJerkLimits()){
        ROS_WARN_STREAM_THROTTLE(1, "jerk limits are not yet implementend");
    }

}

bool JointLimiter::Limits::valid() const {
    // TODO: implement validity checks, e.g. min_position <= max_position
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
    }while(0)

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
        if(hasSoftLimits() && other.hasSoftLimits()){ // merge
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
#define TRY_COPY(flag, max_prop, has_prop)                      \
    do {                                                        \
        if(other.limits_flags & flag){                          \
            joint_limits.max_prop = other.joint_limits.max_prop;\
            joint_limits.has_prop = other.joint_limits.has_prop;\
        }                                                       \
    }while(0)

void JointLimiter::Limits::apply(const Limits &other){
    if(other.limits_flags & PositionLimitsConfigured){
        if(other.hasPositionLimits()){
            setPositionLimits(other.joint_limits.min_position, other.joint_limits.max_position);
        }else{
            joint_limits.has_position_limits = false;
        }
    }
    TRY_COPY(VelocityLimitsConfigured, max_velocity, has_velocity_limits);
    TRY_COPY(AccelerationLimitsConfigured, max_acceleration, has_acceleration_limits);
    TRY_COPY(JerkLimitsConfigured, max_jerk, has_jerk_limits);
    TRY_COPY(EffortLimitsConfigured, max_effort, has_effort_limits);
    if(other.limits_flags & SoftLimitsConfigured){
        soft_limits = other.soft_limits;
        has_soft_limits = other.has_soft_limits;
    }
    limits_flags |= other.limits_flags;
}

std::pair<double,double> JointLimiter::Limits::getVelocitySoftBounds(double pos) const {
    if(!hasSoftLimits()) return std::make_pair(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());

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

void JointLimiter::Limits::setPositionLimits(double min_position, double max_position){
    limits_flags |= PositionLimitsConfigured;
    joint_limits.has_position_limits = true;
    joint_limits.min_position = min_position;
    joint_limits.max_position = max_position;
}

void JointLimiter::Limits::setVelocityLimits(double max_velocity){
    limits_flags |= VelocityLimitsConfigured;
    joint_limits.has_velocity_limits = true;
    joint_limits.max_velocity = max_velocity;
}

void JointLimiter::Limits::setAccelerationLimits(double max_acceleration){
    limits_flags |= AccelerationLimitsConfigured;
    joint_limits.has_acceleration_limits = true;
    joint_limits.max_acceleration = max_acceleration;
}

void JointLimiter::Limits::setJerkLimits(double max_jerk){
    limits_flags |= JerkLimitsConfigured;
    joint_limits.has_jerk_limits = true;
    joint_limits.max_jerk = max_jerk;
}

void JointLimiter::Limits::setEffortLimits(double max_effort){
    limits_flags |= EffortLimitsConfigured;
    joint_limits.has_effort_limits = true;
    joint_limits.max_effort = max_effort;
}

void JointLimiter::Limits::setSoftLimits(double k_position, double min_position, double max_position, double k_velocity){
    limits_flags |= SoftLimitsConfigured;
    has_soft_limits = true;
    soft_limits.k_position = k_position;
    soft_limits.min_position = min_position;
    soft_limits.max_position = max_position;
    soft_limits.k_velocity = k_velocity;
}

std::pair<double, bool> JointLimiter::Limits::limitPositionChecked(double pos) const {
    if(hasPositionLimits()) return limitBoundsChecked(pos, joint_limits.min_position, joint_limits.max_position);
    else return std::make_pair(pos, false);
}

std::pair<double, bool> JointLimiter::Limits::limitVelocityChecked(double vel) const {
    if(hasVelocityLimits()) return limitBoundsChecked(vel, -joint_limits.max_velocity, joint_limits.max_velocity);
    else return std::make_pair(vel, false);
}
std::pair<double, bool> JointLimiter::Limits::limitAccelerationChecked(double acc) const {
    if(hasAccelerationLimits()) return limitBoundsChecked(acc, -joint_limits.max_acceleration, joint_limits.max_acceleration);
    else return std::make_pair(acc, false);
}
std::pair<double, bool> JointLimiter::Limits::limitJerkChecked(double jerk) const {
    if(hasJerkLimits()) return limitBoundsChecked(jerk, -joint_limits.max_jerk, joint_limits.max_jerk);
    else return std::make_pair(jerk, false);
}
std::pair<double, bool> JointLimiter::Limits::limitEffortChecked(double eff) const {
    if(hasEffortLimits()) return limitBoundsChecked(eff, -joint_limits.max_effort, joint_limits.max_effort);
    else return std::make_pair(eff, false);
}

double JointLimiter::Limits::limitPosition(double pos) const {
    return limitPositionChecked(pos).first;
}

double JointLimiter::Limits::limitVelocity(double vel) const {
    return limitVelocityChecked(vel).first;
}
double JointLimiter::Limits::limitAcceleration(double acc) const {
    return limitAccelerationChecked(acc).first;
}
double JointLimiter::Limits::limitJerk(double jerk) const {
    return limitJerkChecked(jerk).first;
}
double JointLimiter::Limits::limitEffort(double eff) const {
    return limitEffortChecked(eff).first;
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

std::pair<double, bool> JointLimiter::Limits::limitVelocityWithSoftBounds(double vel, double current_pos) const {
    std::pair<double, bool> new_soft_vel;

    if(hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = getVelocitySoftBounds(current_pos);
        new_soft_vel = limitBoundsChecked(vel, vel_soft_bounds.first, vel_soft_bounds.second);
    }else{
        new_soft_vel = std::make_pair(vel, false);
    }

    std::pair<double, bool> new_vel = limitVelocityChecked(new_soft_vel.first);
    new_vel.second |= new_soft_vel.second;
    return new_vel;
}

