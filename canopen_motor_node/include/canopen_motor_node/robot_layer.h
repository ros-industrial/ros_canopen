
#ifndef CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_
#define CANOPEN_MOTOR_NODE_ROBOT_LAYER_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>
#include <canopen_402/base.h>
#include <filters/filter_chain.h>
#include <boost/function.hpp>
#include <boost/scoped_ptr.hpp>

#include "muParser.h"

// forward declarations
namespace controller_manager {
    class ControllerManager;
}

class ObjectVariables {
    const boost::shared_ptr<canopen::ObjectStorage> storage_;
    struct Getter {
        boost::shared_ptr<double> val_ptr;
        boost::function<bool(double&)> func;
        bool operator ()() { return func(*val_ptr); }
        template<typename T> Getter(const canopen::ObjectStorage::Entry<T> &entry): func(boost::bind(&Getter::readObject<T>, entry, _1)), val_ptr(new double) { }
        template<typename T> static bool readObject(canopen::ObjectStorage::Entry<T> &entry, double &res){
            T val;
            if(!entry.get(val)) return false;
            res = val;
            return true;
        }
        operator double*() const { return val_ptr.get(); }
    };
    boost::unordered_map<canopen::ObjectDict::Key, Getter> getters_;
public:
    template<const uint16_t dt> static double* func(ObjectVariables &list, const canopen::ObjectDict::Key &key){
        typedef typename canopen::ObjectStorage::DataType<dt>::type type;
        return list.getters_.insert(std::make_pair(key, Getter(list.storage_->entry<type>(key)))).first->second;
    }
    ObjectVariables(const boost::shared_ptr<canopen::ObjectStorage> storage) : storage_(storage) {}
    bool sync(){
        bool ok = true;
        for(boost::unordered_map<canopen::ObjectDict::Key, Getter>::iterator it = getters_.begin(); it != getters_.end(); ++it){
            ok = it->second() && ok;
        }
        return ok;
    }
    double * getVariable(const std::string &n) {
        try{
            if(n.find("obj") == 0){
                canopen::ObjectDict::Key key(n.substr(3));
                boost::unordered_map<canopen::ObjectDict::Key, Getter>::const_iterator it = getters_.find(key);
                if(it != getters_.end()) return it->second;
                return canopen::branch_type<ObjectVariables, double * (ObjectVariables &list, const canopen::ObjectDict::Key &k)>(storage_->dict_->get(key)->data_type)(*this, key);
            }
        }
        catch( const std::exception &e){
            ROS_ERROR_STREAM("Could not find variable '" << n << "', reason: " << boost::diagnostic_information(e));
        }
        return 0;
    }
};

template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_VISIBLE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_OCTET_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_UNICODE_STRING >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }
template<> inline double* ObjectVariables::func<canopen::ObjectDict::DEFTYPE_DOMAIN >(ObjectVariables &, const canopen::ObjectDict::Key &){ return 0; }

class UnitConverter{
public:
    typedef boost::function<double * (const std::string &) > get_var_func_type;

    UnitConverter(const std::string &expression, get_var_func_type var_func = get_var_func_type());
    void reset();
    double evaluate() { int num; return parser_.Eval(num)[0]; }
private:
    typedef boost::shared_ptr<double> variable_ptr;
    typedef std::list<variable_ptr> variable_ptr_list;

    static double* createVariable(const char *name, void * userdata);
    variable_ptr_list var_list_;
    get_var_func_type var_func_;

    mu::Parser parser_;

    static double rad2deg(double r){
        return r*180.0/M_PI;
    }
    static double deg2rad(double d){
        return d*M_PI/180.0;
    }
    static double norm(double val, double min, double max){
        while(val >= max) val -= (max-min);
        while(val < min) val += (max-min);
        return val;
    }
    static double smooth(double val, double old_val, double alpha){
        if(isnan(val)) return 0;
        if(isnan(old_val)) return val;
        return alpha*val + (1.0-alpha)*old_val;
    }
    static double avg(const double *vals, int num)
    {
        double s = 0.0;
        int i=0;
        for (; i<num; ++i){
            const double &val = vals[i];
            if(isnan(val)) break;
            s += val;
        }
        return s / double(i+1);
    }
};

class HandleLayer: public canopen::Layer{
    boost::shared_ptr<canopen::MotorBase> motor_;
    double pos_, vel_, eff_;

    double cmd_pos_, cmd_vel_, cmd_eff_;

    ObjectVariables variables_;
    boost::scoped_ptr<UnitConverter>  conv_target_pos_, conv_target_vel_, conv_target_eff_;
    boost::scoped_ptr<UnitConverter>  conv_pos_, conv_vel_, conv_eff_;

    filters::FilterChain<double> filter_pos_, filter_vel_, filter_eff_;
    XmlRpc::XmlRpcValue options_;

    hardware_interface::JointStateHandle jsh_;
    hardware_interface::JointHandle jph_, jvh_, jeh_;
    boost::atomic<hardware_interface::JointHandle*> jh_;
    boost::atomic<bool> forward_command_;

    typedef boost::unordered_map< canopen::MotorBase::OperationMode,hardware_interface::JointHandle* > CommandMap;
    CommandMap commands_;

    template <typename T> hardware_interface::JointHandle* addHandle( T &iface, hardware_interface::JointHandle *jh,  const std::vector<canopen::MotorBase::OperationMode> & modes){

        bool supported = false;
        for(size_t i=0; i < modes.size(); ++i){
            if(motor_->isModeSupported(modes[i])){
                supported = true;
                break;
            }
        }
        if(!supported) return 0;

        iface.registerHandle(*jh);

        for(size_t i=0; i < modes.size(); ++i){
            commands_[modes[i]] = jh;
        }
        return jh;
    }
    bool select(const canopen::MotorBase::OperationMode &m);
    static double * assignVariable(const std::string &name, double * ptr, const std::string &req) { return name == req ? ptr : 0; }
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<canopen::MotorBase> & motor, const boost::shared_ptr<canopen::ObjectStorage> storage,  XmlRpc::XmlRpcValue & options);

    enum CanSwitchResult{
        NotSupported,
        NotReadyToSwitch,
        ReadyToSwitch,
        NoNeedToSwitch
    };

    CanSwitchResult canSwitch(const canopen::MotorBase::OperationMode &m);
    bool switchMode(const canopen::MotorBase::OperationMode &m);
    bool forwardForMode(const canopen::MotorBase::OperationMode &m);

    void registerHandle(hardware_interface::JointStateInterface &iface){
        iface.registerHandle(jsh_);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface);
    hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface);
    hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface);

    bool prepareFilters(canopen::LayerStatus &status);

private:
    virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleInit(canopen::LayerStatus &status);
    virtual void handleDiag(canopen::LayerReport &report) { /* nothing to do */ }
    virtual void handleShutdown(canopen::LayerStatus &status) { /* nothing to do */ }
    virtual void handleHalt(canopen::LayerStatus &status) { /* TODO */ }
    virtual void handleRecover(canopen::LayerStatus &status) { handleRead(status, Layer::Ready); }

};



class RobotLayer : public canopen::LayerGroupNoDiag<HandleLayer>, public hardware_interface::RobotHW{
    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface pos_interface_;
    hardware_interface::VelocityJointInterface vel_interface_;
    hardware_interface::EffortJointInterface eff_interface_;

    joint_limits_interface::PositionJointSoftLimitsInterface pos_soft_limits_interface_;
    joint_limits_interface::PositionJointSaturationInterface pos_saturation_interface_;
    joint_limits_interface::VelocityJointSoftLimitsInterface vel_soft_limits_interface_;
    joint_limits_interface::VelocityJointSaturationInterface vel_saturation_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface eff_soft_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface eff_saturation_interface_;

    ros::NodeHandle nh_;
    urdf::Model urdf_;

    typedef boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> > HandleMap;
    HandleMap handles_;
    typedef std::vector<std::pair<boost::shared_ptr<HandleLayer>, canopen::MotorBase::OperationMode> >  SwitchContainer;
    typedef boost::unordered_map<std::string, SwitchContainer>  SwitchMap;
    SwitchMap switch_map_;

    boost::atomic<bool> first_init_;

    void stopControllers(const std::vector<std::string> controllers);
public:
    void add(const std::string &name, boost::shared_ptr<HandleLayer> handle);
    RobotLayer(ros::NodeHandle nh);
    boost::shared_ptr<const urdf::Joint> getJoint(const std::string &n) const { return urdf_.getJoint(n); }

    virtual void handleInit(canopen::LayerStatus &status);
    void enforce(const ros::Duration &period, bool reset);
    virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list);
};


class ControllerManagerLayer : public canopen::Layer {
    boost::shared_ptr<controller_manager::ControllerManager> cm_;
    boost::shared_ptr<RobotLayer> robot_;
    ros::NodeHandle nh_;

    canopen::time_point last_time_;
    boost::atomic<bool> recover_;
    const ros::Duration fixed_period_;

public:
    ControllerManagerLayer(const boost::shared_ptr<RobotLayer> robot, const ros::NodeHandle &nh, const ros::Duration &fixed_period)
    :Layer("ControllerManager"), robot_(robot), nh_(nh), fixed_period_(fixed_period) {
    }

    virtual void handleRead(canopen::LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(canopen::LayerStatus &status, const LayerState &current_state);    virtual void handleDiag(canopen::LayerReport &report) { /* nothing to do */ }
    virtual void handleHalt(canopen::LayerStatus &status) { /* nothing to do (?) */ }
    virtual void handleInit(canopen::LayerStatus &status);
    virtual void handleRecover(canopen::LayerStatus &status);
    virtual void handleShutdown(canopen::LayerStatus &status);
};

#endif
