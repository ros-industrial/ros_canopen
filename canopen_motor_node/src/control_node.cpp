#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/socketcan.h>
#include <canopen_chain_node/ros_chain.h>

#include <canopen_motor_node/robot_layer.h>

using namespace can;
using namespace canopen;



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

        canopen::LayerStatus s;
        if(!handle->prepareFilters(s)){
            ROS_ERROR_STREAM(s.reason());
            return false;
        }

        robot_layer_->add(joint, handle);
        logger->add(handle);

        return true;
    }

public:
    MotorChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv): RosChain(nh, nh_priv), motor_allocator_("canopen_402", "canopen::MotorBase::Allocator"){}

    virtual bool setup() {
        motors_.reset( new LayerGroupNoDiag<MotorBase>("402 Layer"));
        robot_layer_.reset( new RobotLayer(nh_));

        ros::Duration dur(0.0) ;

        if(RosChain::setup()){
            add(motors_);
            add(robot_layer_);

            if(!nh_.param("use_realtime_period", false)){
                dur.fromSec(boost::chrono::duration<double>(update_duration_).count());
                ROS_INFO_STREAM("Using fixed control period: " << dur);
            }else{
                ROS_INFO("Using real-time control period");
            }
            cm_.reset(new ControllerManagerLayer(robot_layer_, nh_, dur));
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
      return 1;
  }

  ros::waitForShutdown();
  return 0;
}
