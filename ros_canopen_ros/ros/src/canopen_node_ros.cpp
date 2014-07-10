// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros_canopen_ros/canopen_nodeConfig.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <industrial_msgs/RobotStatus.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>

// other includes
#include <canopen_node_common.cpp>


class canopen_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<ros_canopen_ros::canopen_nodeConfig> server;
    dynamic_reconfigure::Server<ros_canopen_ros::canopen_nodeConfig>::CallbackType f;

    ros::Publisher joint_states_;
    ros::Publisher state_;
    ros::Publisher diagnostics_;
    ros::Publisher robot_status_;
    ros::Subscriber command_vel_;
    ros::Subscriber command_pos_;
    ros::ServiceServer init_;
    ros::ServiceServer recover_;
    ros::ServiceServer shutdown_;
    ros::ServiceServer halt_;

    canopen_node_data component_data_;
    canopen_node_config component_config_;
    canopen_node_impl component_implementation_;

    canopen_node_ros() : np_("~")
    {
        f = boost::bind(&canopen_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string init_remap;
        n_.param("init_remap", init_remap, (std::string)"init");
        init_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(init_remap, boost::bind(&canopen_node_impl::callback_init, &component_implementation_,_1,_2,component_config_));
        std::string recover_remap;
        n_.param("recover_remap", recover_remap, (std::string)"recover");
        recover_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(recover_remap, boost::bind(&canopen_node_impl::callback_recover, &component_implementation_,_1,_2,component_config_));
        std::string shutdown_remap;
        n_.param("shutdown_remap", shutdown_remap, (std::string)"shutdown");
        shutdown_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(shutdown_remap, boost::bind(&canopen_node_impl::callback_shutdown, &component_implementation_,_1,_2,component_config_));
        std::string halt_remap;
        n_.param("halt_remap", halt_remap, (std::string)"halt");
        halt_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(halt_remap, boost::bind(&canopen_node_impl::callback_halt, &component_implementation_,_1,_2,component_config_));

        joint_states_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
        state_ = n_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
        diagnostics_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
        robot_status_ = n_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);
        command_vel_ = n_.subscribe("command_vel", 1, &canopen_node_ros::topicCallback_command_vel, this);
        command_pos_ = n_.subscribe("command_pos", 1, &canopen_node_ros::topicCallback_command_pos, this);

        np_.param("diagnostics_frequency", component_config_.diagnostics_frequency, (double)1);
        np_.param("chain_name", component_config_.chain_name, (std::string)"");
        np_.param("timeout", component_config_.timeout, (double)0.1);
        if(np_.hasParam("bus"))
            np_.getParam("bus", component_config_.bus);
        else
            ROS_ERROR("Parameter bus not set");
        if(np_.hasParam("moduls"))
            np_.getParam("moduls", component_config_.moduls);
        else
            ROS_ERROR("Parameter moduls not set");
        if(np_.hasParam("robot_description"))
            np_.getParam("robot_description", component_config_.robot_description);
        else
            ROS_ERROR("Parameter robot_description not set");
        }





    void topicCallback_command_vel(const brics_actuator::JointVelocities::ConstPtr& msg)
    {
        component_data_.in_command_vel = *msg;
        update(); //call only if defined as event port, not called if it is an data port
    }
    void topicCallback_command_pos(const brics_actuator::JointPositions::ConstPtr& msg)
    {
        component_data_.in_command_pos = *msg;
        update(); //call only if defined as event port, not called if it is an data port
    }

    void configure_callback(ros_canopen_ros::canopen_nodeConfig &config, uint32_t level)
    {
        component_config_.diagnostics_frequency = config.diagnostics_frequency;
        component_config_.chain_name = config.chain_name;
        component_config_.timeout = config.timeout;
    }

    void configure()
    {
        component_implementation_.configure(component_config_);
    }

    void activate_all_output()
    {
        component_data_.out_joint_states_active = true;
        component_data_.out_state_active = true;
        component_data_.out_diagnostics_active = true;
        component_data_.out_robot_status_active = true;
    }

    void update()
    {
        activate_all_output();
        component_implementation_.update(component_data_, component_config_);
        if (component_data_.out_joint_states_active)
            joint_states_.publish(component_data_.out_joint_states);
        if (component_data_.out_state_active)
            state_.publish(component_data_.out_state);
        if (component_data_.out_diagnostics_active)
            diagnostics_.publish(component_data_.out_diagnostics);
        if (component_data_.out_robot_status_active)
            robot_status_.publish(component_data_.out_robot_status);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "canopen_node");

    canopen_node_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
