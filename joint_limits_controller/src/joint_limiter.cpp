#include <joint_limits_controller/limited_joint_handle.h>

double JointLimiter::limitBounds(double value, double min, double max){
    if(min > max){
        ROS_ERROR("min > max");
        return std::numeric_limits<double>::quiet_NaN();
    }
    if(value > max) return max;
    else if (value < min) return min;
    return value;
}

std::pair<double,double> JointLimiter::getSoftBounds(double value, double k, double lower, double upper){
    return std::make_pair(-k*(value-lower), -k*(value-upper));
}

void PositionJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double last_command = 0;

    if(!last_command_.get(last_command)) last_command = pos; // fallback to actual position

    if(limits.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits.getVelocitySoftBounds(last_command);
        cmd = limitBounds(cmd, last_command + vel_soft_bounds.first * period, last_command + vel_soft_bounds.second * period);
    }

    double max_vel;
    if(limits.getVelocityLimit(max_vel, period)){
        cmd = limitBounds(cmd, last_command - max_vel * period, last_command + max_vel * period);
    }

    cmd = limits.limitPosititon(cmd);

    //TODO: What do to if effort limit is exceeded?

    last_command_.set(cmd);
}

void VelocityJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double last_command = 0;

    if(!last_command_.get(last_command)) last_command = vel; // fallback to actual velocity

    if(limits.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits.getVelocitySoftBounds(pos); // TODO: use tracked pos?
        cmd = limitBounds(cmd, vel_soft_bounds.first, vel_soft_bounds.second);
    }

    double max_vel;
    if(limits.getVelocityLimit(max_vel, period)){
        cmd = limitBounds(cmd, -max_vel, max_vel);
    }

    cmd = limits.stopOnPositionLimit(cmd, pos);

    //TODO: What do to if effort limit is exceeded?

    last_command_.set(cmd);
}

void EffortJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double last_command = 0;

    if(!last_command_.get(last_command)) last_command = eff; // fallback to actual effort

    if(limits.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits.getVelocitySoftBounds(pos); // TODO: use tracked pos?

        std::pair<double, double> eff_soft_bounds = getSoftBounds(vel, limits.soft_limits.k_velocity, vel_soft_bounds.first, vel_soft_bounds.second);

        eff_soft_bounds.first = limits.limitEffort(eff_soft_bounds.first);
        eff_soft_bounds.second = limits.limitEffort(eff_soft_bounds.second);

        cmd = limitBounds(cmd, eff_soft_bounds.first, eff_soft_bounds.second);
    }

    cmd = limits.limitEffort(cmd);

    cmd = limits.stopOnPositionLimit(cmd, pos);

    last_command_.set(cmd);
}
