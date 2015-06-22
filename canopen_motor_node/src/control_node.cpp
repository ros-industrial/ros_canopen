#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/chain_ros.h>

#include <canopen_402/base.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <urdf/model.h>

#include <controller_manager/controller_manager.h>

#include "muParser.h"

using namespace can;
using namespace canopen;

class UnitConverter{
public:
    typedef boost::function<double * (const std::string &) > get_var_func_type;

    UnitConverter(const std::string &expression, get_var_func_type var_func = get_var_func_type())
    : var_func_(var_func)
    {
        parser_.SetVarFactory(UnitConverter::createVariable, this);

        parser_.DefineConst("pi", M_PI);
        parser_.DefineConst("nan", std::numeric_limits<double>::quiet_NaN());

        parser_.DefineFun("rad2deg", UnitConverter::rad2deg);
        parser_.DefineFun("deg2rad", UnitConverter::deg2rad);
        parser_.DefineFun("norm", UnitConverter::norm);
        parser_.DefineFun("smooth", UnitConverter::smooth);
        parser_.DefineFun("avg", UnitConverter::avg);

        parser_.SetExpr(expression);
    }
    void reset(){
        for(variable_ptr_list::iterator it = var_list_.begin(); it != var_list_.end(); ++it){
            **it = std::numeric_limits<double>::quiet_NaN();
        }
    }
    double evaluate() { int num; return parser_.Eval(num)[0]; }
private:
    typedef boost::shared_ptr<double> variable_ptr;
    typedef std::list<variable_ptr> variable_ptr_list;

    static double* createVariable(const char *name, void * userdata){
        UnitConverter * uc = static_cast<UnitConverter*>(userdata);
        double *p = uc->var_func_ ? uc->var_func_(name) : 0;
        if(!p){
            p = new double(std::numeric_limits<double>::quiet_NaN());
            uc->var_list_.push_back(variable_ptr(p));
        }
        return p;
    }
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

class ObjectVariables {
    const boost::shared_ptr<ObjectStorage> storage_;
    struct Getter {
        boost::shared_ptr<double> val_ptr;
        boost::function<bool(double&)> func;
        bool operator ()() { return func(*val_ptr); }
        template<typename T> Getter(const ObjectStorage::Entry<T> &entry): func(boost::bind(&Getter::readObject<T>, entry, _1)), val_ptr(new double) { }
        template<typename T> static bool readObject(ObjectStorage::Entry<T> &entry, double &res){
            T val;
            if(!entry.get(val)) return false;
            res = val;
            return true;
        }
        operator double*() const { return val_ptr.get(); }
    };
    boost::unordered_map<ObjectDict::Key, Getter> getters_;
public:
    template<const uint16_t dt> static double* func(ObjectVariables &list, const ObjectDict::Key &key){
        typedef typename ObjectStorage::DataType<dt>::type type;
        return list.getters_.insert(std::make_pair(key, Getter(list.storage_->entry<type>(key)))).first->second;
    }
    ObjectVariables(const boost::shared_ptr<ObjectStorage> storage) : storage_(storage) {}
    bool sync(){
        bool ok = true;
        for(boost::unordered_map<ObjectDict::Key, Getter>::iterator it = getters_.begin(); it != getters_.end(); ++it){
            ok = it->second() && ok;
        }
        return ok;
    }
    double * getVariable(const std::string &n) {
        try{
            if(n.find("obj") == 0){
                ObjectDict::Key key(n.substr(3));
                boost::unordered_map<ObjectDict::Key, Getter>::const_iterator it = getters_.find(key);
                if(it != getters_.end()) return it->second;
                return branch_type<ObjectVariables, double * (ObjectVariables &list, const ObjectDict::Key &k)>(storage_->dict_->get(key)->data_type)(*this, key);
            }
        }
        catch( const std::exception &e){
            ROS_ERROR_STREAM("Could not find variable '" << n << "', reason: " << boost::diagnostic_information(e));
        }
        return 0;
    }
};

template<> double* ObjectVariables::func<ObjectDict::DEFTYPE_VISIBLE_STRING >(ObjectVariables &, const ObjectDict::Key &){ return 0; }
template<> double* ObjectVariables::func<ObjectDict::DEFTYPE_OCTET_STRING >(ObjectVariables &, const ObjectDict::Key &){ return 0; }
template<> double* ObjectVariables::func<ObjectDict::DEFTYPE_UNICODE_STRING >(ObjectVariables &, const ObjectDict::Key &){ return 0; }
template<> double* ObjectVariables::func<ObjectDict::DEFTYPE_DOMAIN >(ObjectVariables &, const ObjectDict::Key &){ return 0; }

class HandleLayer: public Layer{
    boost::shared_ptr<MotorBase> motor_;
    double pos_, vel_, eff_;

    double cmd_pos_, cmd_vel_, cmd_eff_;

    ObjectVariables variables_;
    boost::scoped_ptr<UnitConverter>  conv_target_pos_, conv_target_vel_, conv_target_eff_;
    boost::scoped_ptr<UnitConverter>  conv_pos_, conv_vel_, conv_eff_;

    hardware_interface::JointStateHandle jsh_;
    hardware_interface::JointHandle jph_, jvh_, jeh_;
    boost::atomic<hardware_interface::JointHandle*> jh_;

    typedef boost::unordered_map< MotorBase::OperationMode,hardware_interface::JointHandle* > CommandMap;
    CommandMap commands_;

    template <typename T> hardware_interface::JointHandle* addHandle( T &iface, hardware_interface::JointHandle *jh,  const std::vector<MotorBase::OperationMode> & modes){

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
    bool select(const MotorBase::OperationMode &m){
        CommandMap::iterator it = commands_.find(m);
        if(it == commands_.end()) return false;
        jh_ = it->second;
        return true;
    }
    static double * assignVariable(const std::string &name, double * ptr, const std::string &req) { return name == req ? ptr : 0; }
public:
    HandleLayer(const std::string &name, const boost::shared_ptr<MotorBase> & motor, const boost::shared_ptr<ObjectStorage> storage,  XmlRpc::XmlRpcValue & options)
    : Layer(name + " Handle"), motor_(motor), variables_(storage), jsh_(name, &pos_, &vel_, &eff_), jph_(jsh_, &cmd_pos_), jvh_(jsh_, &cmd_vel_), jeh_(jsh_, &cmd_eff_), jh_(0) {
       commands_[MotorBase::No_Mode] = 0;

       std::string p2d("rint(rad2deg(pos)*1000)"), v2d("rint(rad2deg(vel)*1000)"), e2d("rint(eff)");
       std::string p2r("deg2rad(obj6064)/1000"), v2r("deg2rad(obj606C)/1000"), e2r("0");

       if(options.hasMember("pos_unit_factor") || options.hasMember("vel_unit_factor") || options.hasMember("eff_unit_factor")){
           const std::string reason("*_unit_factor parameters are not supported anymore, please migrate to conversion functions.");
           ROS_FATAL_STREAM(reason);
           throw std::invalid_argument(reason);
       }

       if(options.hasMember("pos_to_device")) p2d = (const std::string&) options["pos_to_device"];
       if(options.hasMember("pos_from_device")) p2r = (const std::string&) options["pos_from_device"];

       if(options.hasMember("vel_to_device")) v2d = (const std::string&) options["vel_to_device"];
       if(options.hasMember("vel_from_device")) v2r = (const std::string&) options["vel_from_device"];

       if(options.hasMember("eff_to_device")) e2d = (const std::string&) options["eff_to_device"];
       if(options.hasMember("eff_from_device")) e2r = (const std::string&) options["eff_from_device"];

       conv_target_pos_.reset(new UnitConverter(p2d, boost::bind(assignVariable, "pos", &cmd_pos_, _1)));
       conv_target_vel_.reset(new UnitConverter(v2d, boost::bind(assignVariable, "vel", &cmd_vel_, _1)));
       conv_target_eff_.reset(new UnitConverter(e2d, boost::bind(assignVariable, "eff", &cmd_eff_, _1)));

       conv_pos_.reset(new UnitConverter(p2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
       conv_vel_.reset(new UnitConverter(v2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
       conv_eff_.reset(new UnitConverter(e2r, boost::bind(&ObjectVariables::getVariable, &variables_, _1)));
    }

    int canSwitch(const MotorBase::OperationMode &m){
       if(!motor_->isModeSupported(m)) return 0;
       if(motor_->getMode() == m) return -1;
       if(commands_.find(m) != commands_.end()) return 1;
       return 0;
    }
    bool switchMode(const MotorBase::OperationMode &m){
        jh_ = 0; // disconnect handle
        if(!motor_->enterModeAndWait(m)){
            ROS_ERROR_STREAM(jsh_.getName() << "could not enter mode " << (int)m);
        }
        return select(m);
    }

    void registerHandle(hardware_interface::JointStateInterface &iface){
        iface.registerHandle(jsh_);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::PositionJointInterface &iface){
        std::vector<MotorBase::OperationMode> modes;
        modes.push_back(MotorBase::Profiled_Position);
        modes.push_back(MotorBase::Interpolated_Position);
        modes.push_back(MotorBase::Cyclic_Synchronous_Position);
        return addHandle(iface, &jph_, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::VelocityJointInterface &iface){
        std::vector<MotorBase::OperationMode> modes;
        modes.push_back(MotorBase::Velocity);
        modes.push_back(MotorBase::Profiled_Velocity);
        modes.push_back(MotorBase::Cyclic_Synchronous_Velocity);
        return addHandle(iface, &jvh_, modes);
    }
    hardware_interface::JointHandle* registerHandle(hardware_interface::EffortJointInterface &iface){
        std::vector<MotorBase::OperationMode> modes;
        modes.push_back(MotorBase::Profiled_Torque);
        modes.push_back(MotorBase::Cyclic_Synchronous_Torque);
        return addHandle(iface, &jeh_, modes);
    }
private:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Shutdown){
            variables_.sync();
            pos_ = conv_pos_->evaluate();
            vel_ = conv_vel_->evaluate();
            eff_ = conv_eff_->evaluate();
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state == Ready){
            hardware_interface::JointHandle* jh = jh_;
            if(jh_ == &jph_){
                motor_->setTarget(conv_target_pos_->evaluate());
                cmd_vel_ = vel_;
                cmd_eff_ = eff_;
            }else if(jh_ == &jvh_){
                motor_->setTarget(conv_target_vel_->evaluate());
                cmd_pos_ = pos_;
                cmd_eff_ = eff_;
            }else if(jh_ == &jeh_){
                motor_->setTarget(conv_target_eff_->evaluate());
                cmd_pos_ = pos_;
                cmd_vel_ = vel_;
            }else{
                cmd_pos_ = pos_;
                cmd_vel_ = vel_;
                cmd_eff_ = eff_;
                if(jh_) status.warn("unsupported mode active");
            }
        }
    }
    virtual void handleInit(LayerStatus &status){
        // TODO: implement proper init
        conv_pos_->reset();
        conv_vel_->reset();
        conv_eff_->reset();
        conv_target_pos_->reset();
        conv_target_vel_->reset();
        conv_target_eff_->reset();
        handleRead(status, Layer::Ready);
    }
    
    virtual void handleDiag(LayerReport &report) { /* nothing to do */ }
    virtual void handleShutdown(LayerStatus &status) { /* nothing to do */ }
    virtual void handleHalt(LayerStatus &status) { /* TODO */ }
    virtual void handleRecover(LayerStatus &status) { handleRead(status, Layer::Ready); }
    
};


class RobotLayer : public LayerGroupNoDiag<HandleLayer>, public hardware_interface::RobotHW{
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
    typedef std::vector<std::pair<boost::shared_ptr<HandleLayer>, MotorBase::OperationMode> >  SwitchContainer;
    typedef boost::unordered_map<std::string, SwitchContainer>  SwitchMap;
    mutable SwitchMap switch_map_;

public:

    void add(const std::string &name, boost::shared_ptr<HandleLayer> handle){
        LayerGroupNoDiag::add(handle);
        handles_.insert(std::make_pair(name, handle));
    }
    RobotLayer(ros::NodeHandle nh) : LayerGroupNoDiag<HandleLayer>("RobotLayer"), nh_(nh)
    {
        registerInterface(&state_interface_);
        registerInterface(&pos_interface_);
        registerInterface(&vel_interface_);
        registerInterface(&eff_interface_);


        registerInterface(&pos_saturation_interface_);
        registerInterface(&pos_soft_limits_interface_);
        registerInterface(&vel_saturation_interface_);
        registerInterface(&vel_soft_limits_interface_);
        registerInterface(&eff_saturation_interface_);
        registerInterface(&eff_soft_limits_interface_);

        urdf_.initParam("robot_description");
    }

    boost::shared_ptr<const urdf::Joint> getJoint(const std::string &n) const { return urdf_.getJoint(n); }

    virtual void handleInit(LayerStatus &status){
        urdf::Model urdf;
        urdf.initParam("robot_description");

        for(HandleMap::iterator it = handles_.begin(); it != handles_.end(); ++it){
            joint_limits_interface::JointLimits limits;
            joint_limits_interface::SoftJointLimits soft_limits;

            boost::shared_ptr<const urdf::Joint> joint = getJoint(it->first);
            
            if(!joint){
                status.error("joint " + it->first + " not found");
                return;
            }

            bool has_joint_limits = joint_limits_interface::getJointLimits(joint, limits);

            has_joint_limits = joint_limits_interface::getJointLimits(it->first, nh_, limits) || has_joint_limits;

            bool has_soft_limits = has_joint_limits && joint_limits_interface::getSoftJointLimits(joint, soft_limits);

            if(!has_joint_limits){
                ROS_WARN_STREAM("No limits found for " << it->first);
            }

            it->second->registerHandle(state_interface_);

            const hardware_interface::JointHandle *h  = 0;

            h = it->second->registerHandle(pos_interface_);
            if(h && limits.has_position_limits){
                joint_limits_interface::PositionJointSaturationHandle sathandle(*h, limits);
                pos_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::PositionJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    pos_soft_limits_interface_.registerHandle(softhandle);
                }
            }
            h = it->second->registerHandle(vel_interface_);
            if(h && limits.has_velocity_limits){
                joint_limits_interface::VelocityJointSaturationHandle sathandle(*h, limits);
                vel_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::VelocityJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    vel_soft_limits_interface_.registerHandle(softhandle);
                }
            }
            h = it->second->registerHandle(eff_interface_);
            if(h && limits.has_effort_limits){
                joint_limits_interface::EffortJointSaturationHandle sathandle(*h, limits);
                eff_saturation_interface_.registerHandle(sathandle);
                if(has_soft_limits){
                    joint_limits_interface::EffortJointSoftLimitsHandle softhandle(*h, limits,soft_limits);
                    eff_soft_limits_interface_.registerHandle(softhandle);
                }
            }

        }
        LayerGroupNoDiag::handleInit(status);
    }

    void enforce(const ros::Duration &period, bool reset){
        if(reset){
            pos_saturation_interface_.reset();
            pos_soft_limits_interface_.reset();
        }
        pos_saturation_interface_.enforceLimits(period);
        pos_soft_limits_interface_.enforceLimits(period);
        vel_saturation_interface_.enforceLimits(period);
        vel_soft_limits_interface_.enforceLimits(period);
        eff_saturation_interface_.enforceLimits(period);
        eff_soft_limits_interface_.enforceLimits(period);
    }
    virtual bool canSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) const {

        (void) &hardware_interface::RobotHW::canSwitch; // compile-time check for recent ros-controls version of ros_control

        // stop handles
        for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){

            if(switch_map_.find(controller_it->name) == switch_map_.end()){
                ROS_ERROR_STREAM(controller_it->name << " was not started before");
                return false;
            }
        }

        // start handles
        for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
            if(switch_map_.find(controller_it->name) != switch_map_.end()) continue;

            SwitchContainer to_switch;
            ros::NodeHandle nh(nh_,controller_it->name);
            int mode;
            if(nh.getParam("required_drive_mode", mode)){


                for (std::set<std::string>::const_iterator res_it = controller_it->resources.begin(); res_it != controller_it->resources.end(); ++res_it){
                    boost::unordered_map< std::string, boost::shared_ptr<HandleLayer> >::const_iterator h_it = handles_.find(*res_it);

                    if(h_it == handles_.end()){
                        ROS_ERROR_STREAM(*res_it << " not found");
                        return false;
                    }
                    if(int res = h_it->second->canSwitch((MotorBase::OperationMode)mode)){
                        if(res > 0) to_switch.push_back(std::make_pair(h_it->second, MotorBase::OperationMode(mode)));
                    }else{
                        ROS_ERROR_STREAM("Mode " << mode << " is not available for " << *res_it);
                        return false;
                    }
                }
            }
            switch_map_.insert(std::make_pair(controller_it->name, to_switch));
        }
        return true;
    }

    virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list, const std::list<hardware_interface::ControllerInfo> &stop_list) {
        boost::unordered_set<boost::shared_ptr<HandleLayer> > to_stop;
        for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = stop_list.begin(); controller_it != stop_list.end(); ++controller_it){
            SwitchContainer &to_switch = switch_map_.at(controller_it->name);
            for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
                to_stop.insert(it->first);
            }
        }
        for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it = start_list.begin(); controller_it != start_list.end(); ++controller_it){
            SwitchContainer &to_switch = switch_map_.at(controller_it->name);
            for(RobotLayer::SwitchContainer::iterator it = to_switch.begin(); it != to_switch.end(); ++it){
                it->first->switchMode(it->second);
                to_stop.erase(it->first);
            }
        }
        for(boost::unordered_set<boost::shared_ptr<HandleLayer> >::iterator it = to_stop.begin(); it != to_stop.end(); ++it){
            (*it)->switchMode(MotorBase::No_Mode);
        }
    }

};

class ControllerManagerLayer : public Layer {
    boost::shared_ptr<controller_manager::ControllerManager> cm_;
    boost::shared_ptr<RobotLayer> robot_;
    ros::NodeHandle nh_;

    canopen::time_point last_time_;
    boost::atomic<bool> recover_;

public:
    ControllerManagerLayer(const boost::shared_ptr<RobotLayer> robot, const ros::NodeHandle &nh)
    :Layer("ControllerManager"), robot_(robot), nh_(nh) {
    }

    virtual void handleRead(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Shutdown){
            if(!cm_) status.error("controller_manager is not intialized");
        }
    }
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state) {
        if(current_state > Shutdown){
            if(!cm_){
                status.error("controller_manager is not intialized");
            }else{
                time_point abs_now = get_abs_time();
                ros::Time now = ros::Time::now();

                ros::Duration period(boost::chrono::duration<double>(abs_now -last_time_).count());
                last_time_ = abs_now;

                bool recover = recover_.exchange(false);
                cm_->update(now, period, recover);
                robot_->enforce(period, recover);
            }
        }
    }
    virtual void handleDiag(LayerReport &report) { /* nothing to do */ }
    virtual void handleHalt(LayerStatus &status) { /* nothing to do (?) */ }

    virtual void handleInit(LayerStatus &status) {
        if(cm_){
            status.warn("controller_manager is already intialized");
        }else{
            recover_ = true;
            cm_.reset(new controller_manager::ControllerManager(robot_.get(), nh_));
        }
    }
    virtual void handleRecover(LayerStatus &status) {
        if(!cm_) status.error("controller_manager is not intialized");
        else recover_ = true;
    }
    virtual void handleShutdown(LayerStatus &status) {
        cm_.reset();
    }
};

class XmlRpcSettings : public Settings{
public:
    XmlRpcSettings() {}
    XmlRpcSettings(const XmlRpc::XmlRpcValue &v) : value_(v) {}
    XmlRpcSettings& operator=(const XmlRpc::XmlRpcValue &v) { value_ = v; return *this; }
private:
    virtual bool getRepr(const std::string &n, std::string & repr) const {
        if(value_.hasMember(n)){
            std::stringstream sstr;
            sstr << const_cast< XmlRpc::XmlRpcValue &>(value_)[n]; // does not write since already existing
            repr = sstr.str();
            return true;
        }
        return false;
    }
    XmlRpc::XmlRpcValue value_;

};

class MotorChain : public RosChain{
    ClassAllocator<canopen::MotorBase> motor_allocator_;
    boost::shared_ptr< LayerGroupNoDiag<MotorBase> > motors_;
    boost::shared_ptr<RobotLayer> robot_layer_;

    boost::shared_ptr< ControllerManagerLayer> cm_;

    virtual bool nodeAdded(XmlRpc::XmlRpcValue &params, const boost::shared_ptr<canopen::Node> &node, const boost::shared_ptr<Logger> &logger)
    {
        std::string name = params["name"];
        std::string &joint = name;
        if(params.hasMember("joint")) joint.assign(params["joint"]);

        if(!robot_layer_->getJoint(joint)){
            ROS_ERROR_STREAM("joint " + joint + " was not found in URDF");
            return false;
        }

        std::string alloc_name = "canopen::Motor402::Allocator";
        if(params.hasMember("motor_allocator")) alloc_name.assign(params["motor_allocator"]);

        XmlRpcSettings settings;
        if(params.hasMember("motor_layer")) settings = params["motor_layer"];

        boost::shared_ptr<MotorBase> motor;

        try{
            motor = motor_allocator_.allocateInstance(alloc_name, name + "_motor", node->getStorage(), settings);
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            return false;
        }

        if(!motor){
            ROS_ERROR_STREAM("Could not allocate motor.");
            return false;
        }

        motor->registerDefaultModes(node->getStorage());
        motors_->add(motor);
        logger->add(motor);

        boost::shared_ptr<HandleLayer> handle( new HandleLayer(joint, motor, node->getStorage(), params));
        robot_layer_->add(joint, handle);
        logger->add(handle);

        return true;
    }

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv), motor_allocator_("canopen_402", "canopen::MotorBase::Allocator"){}

    virtual bool setup() {
        motors_.reset( new LayerGroupNoDiag<MotorBase>("402 Layer"));
        robot_layer_.reset( new RobotLayer(nh_));
        cm_.reset(new ControllerManagerLayer(robot_layer_, nh_));

        if(RosChain::setup()){
            add(motors_);
            add(robot_layer_);

            add(cm_);

            return true;
        }

        return false;
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "canopen_chain_node_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  MotorChain chain(nh, nh_priv);

  if(!chain.setup()){
      return -1;
  }

  ros::waitForShutdown();
  return 0;
}
