#include <joint_limits_controller/limited_joint_handle.h>

std::pair<double,bool> JointLimiter::Limits::limitBoundsChecked(double value, double min, double max){
    if(min > max){
        ROS_ERROR("min > max");
        return std::make_pair(std::numeric_limits<double>::quiet_NaN(), false);
    }
    if(value > max) return std::make_pair(max, true);
    else if (value < min) return std::make_pair(min, true);
    return std::make_pair(value, false);
}

double JointLimiter::Limits::limitBounds(double value, double min, double max){
    return limitBoundsChecked(value, min, max).first;
}

std::pair<double,double> JointLimiter::Limits::getSoftBounds(double value, double k, double lower, double upper){
    return std::make_pair(-k*(value-lower), -k*(value-upper));
}

void PositionJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double current_pos = pos_.getOrInit(pos);

    std::pair<double, bool> new_vel = limits.limitVelocityWithSoftBounds((cmd - current_pos)/period, current_pos);

    if(new_vel.second)
    {
        cmd = current_pos + new_vel.first * period;
    }

    cmd = limits.limitPosition(cmd);

    pos_.set(cmd);

    //TODO: What do to if effort limit is exceeded?
}

void VelocityJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double current_vel = vel_.getOrInit(vel);

    std::pair<double, bool> new_acc = limits.limitAccelerationChecked((cmd - current_vel)/period);
    if(new_acc.second)
    {
        cmd = current_vel + new_acc.first * period;
    }

    std::pair<double, bool>  new_vel = limits.limitVelocityWithSoftBounds(cmd, pos + cmd*period);
    cmd = limits.stopOnPositionLimit(new_vel.first, pos + cmd*period);
    vel_.set(cmd);

    //TODO: What do to if effort limit is exceeded?
}

void EffortJointLimiter::enforceLimits(const double& period, const Limits &limits, const double pos, const double vel, const double eff, double &cmd) {
    double current_eff = eff_.getOrInit(eff);

    if(limits.hasSoftLimits()){
        std::pair<double, double> vel_soft_bounds = limits.getVelocitySoftBounds(pos); // TODO: use tracked pos?

        std::pair<double, double> eff_soft_bounds = Limits::getSoftBounds(vel, limits.soft_limits.k_velocity, vel_soft_bounds.first, vel_soft_bounds.second);

        eff_soft_bounds.first = limits.limitEffort(eff_soft_bounds.first);
        eff_soft_bounds.second = limits.limitEffort(eff_soft_bounds.second);

        cmd = Limits::limitBounds(cmd, eff_soft_bounds.first, eff_soft_bounds.second);
    }

    cmd = limits.limitEffort(cmd);

    cmd = limits.stopOnPositionLimit(cmd, pos);

    eff_.set(cmd);
}
