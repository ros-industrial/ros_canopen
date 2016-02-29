#include <joint_limits_controller/limited_joint_handle.h>

double LimitedJointHandle::limitBounds(double value, double min, double max){
    if(value > max) return max;
    else if (value < min) return min;
    return value;
}

std::pair<double,double> LimitedJointHandle::getSoftBounds(double value, double k, double lower, double upper){
    return std::make_pair(-k*(value-lower), -k*(value-upper));
}

void LimitedPositionJointHandle::enforceLimits(const ros::Duration& period) {
    double cmd = getCommand(), last_command = 0;

    if(!last_command_.get(last_command)) last_command = getPosition(); // fallback to actual position

    if(limits_.hasSoftLimits()){
        std::pair<double, double> soft_bounds = limits_.getVelocitySoftBounds(last_command);

        // try to limit velocity to hard bounds
        soft_bounds.first = limits_.limitVelocity(soft_bounds.first);
        soft_bounds.second = limits_.limitVelocity(soft_bounds.second);

        cmd = limitBounds(cmd, last_command + soft_bounds.first * period.toSec(), last_command + soft_bounds.second * period.toSec());
    }

    double max_vel;
    if(limits_.getVelocityLimit(max_vel, period)){
        cmd = limitBounds(cmd, last_command - max_vel * period.toSec(), last_command + max_vel * period.toSec());
    }

    cmd = limits_.limitPosititon(cmd);

    //TODO: What do to if effort limit is exceeded?

    last_command_.set(cmd);
    setCommand(cmd);
}

void LimitedVelocityJointHandle::enforceLimits(const ros::Duration& period) {
    double cmd = getCommand(), last_command = 0, current_pos = getPosition();

    if(!last_command_.get(last_command)) last_command = getVelocity(); // fallback to actual velocity

    if(limits_.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits_.getVelocitySoftBounds(current_pos); // TODO: use tracked pos?

        cmd = limitBounds(cmd, vel_soft_bounds.first, vel_soft_bounds.second);

    }

    double max_vel;
    if(limits_.getVelocityLimit(max_vel, period)){
        cmd = limitBounds(cmd, -max_vel, max_vel);
    }

    cmd = limits_.stopOnPositionLimit(cmd, current_pos);

    //TODO: What do to if effort limit is exceeded?

    last_command_.set(cmd);
    setCommand(cmd);
}

void LimitedEffortJointHandle::enforceLimits(const ros::Duration& period) {
    double cmd = getCommand(), last_command = 0, current_pos = getPosition();

    if(!last_command_.get(last_command)) last_command = getEffort(); // fallback to actual effort

    if(limits_.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits_.getVelocitySoftBounds(current_pos); // TODO: use tracked pos?

        std::pair<double, double> eff_soft_bounds = getSoftBounds(getVelocity(), limits_.soft_limits.k_velocity, vel_soft_bounds.first, vel_soft_bounds.second);

        eff_soft_bounds.first = limits_.limitEffort(eff_soft_bounds.first);
        eff_soft_bounds.second = limits_.limitEffort(eff_soft_bounds.second);

        cmd = limitBounds(cmd, eff_soft_bounds.first, eff_soft_bounds.second);
    }

    cmd = limits_.limitEffort(cmd);

    cmd = limits_.stopOnPositionLimit(cmd, current_pos);

    last_command_.set(cmd);
    setCommand(cmd);
}
