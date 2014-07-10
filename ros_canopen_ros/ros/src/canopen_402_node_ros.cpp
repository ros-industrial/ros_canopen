// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros_canopen_ros/canopen_402_nodeConfig.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <industrial_msgs/RobotStatus.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointPositions.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>
#include <industrial_msgs/StopMotion.h>

// other includes
#include <canopen_402_node_common.cpp>


class canopen_402_node_ros
{
    public:
    ros::NodeHandle n_;
    ros::NodeHandle np_;

    dynamic_reconfigure::Server<ros_canopen_ros::canopen_402_nodeConfig> server;
    dynamic_reconfigure::Server<ros_canopen_ros::canopen_402_nodeConfig>::CallbackType f;

    ros::Publisher joint_states_;
    ros::Publisher state_;
    ros::Publisher diagnostics_;
    ros::Publisher robot_status_;
    ros::Publisher feedback_states_;
    ros::Subscriber command_vel_;
    ros::Subscriber command_pos_;
    ros::Subscriber joint_path_command_;
    ros::Subscriber joint_command_;
    ros::ServiceServer init_;
    ros::ServiceServer recover_;
    ros::ServiceServer shutdown_;
    ros::ServiceServer halt_;
    ros::ServiceServer stop_motion_;

    canopen_402_node_data component_data_;
    canopen_402_node_config component_config_;
    canopen_402_node_impl component_implementation_;

    canopen_402_node_ros() : np_("~")
    {
        f = boost::bind(&canopen_402_node_ros::configure_callback, this, _1, _2);
        server.setCallback(f);

        std::string init_remap;
        n_.param("init_remap", init_remap, (std::string)"init");
        init_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(init_remap, boost::bind(&canopen_402_node_impl::callback_init, &component_implementation_,_1,_2,component_config_));
        std::string recover_remap;
        n_.param("recover_remap", recover_remap, (std::string)"recover");
        recover_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(recover_remap, boost::bind(&canopen_402_node_impl::callback_recover, &component_implementation_,_1,_2,component_config_));
        std::string shutdown_remap;
        n_.param("shutdown_remap", shutdown_remap, (std::string)"shutdown");
        shutdown_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(shutdown_remap, boost::bind(&canopen_402_node_impl::callback_shutdown, &component_implementation_,_1,_2,component_config_));
        std::string halt_remap;
        n_.param("halt_remap", halt_remap, (std::string)"halt");
        halt_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(halt_remap, boost::bind(&canopen_402_node_impl::callback_halt, &component_implementation_,_1,_2,component_config_));
        std::string stop_motion_remap;
        n_.param("stop_motion_remap", stop_motion_remap, (std::string)"stop_motion");
        stop_motion_ = n_.advertiseService<industrial_msgs::StopMotion::Request , industrial_msgs::StopMotion::Response>(stop_motion_remap, boost::bind(&canopen_402_node_impl::callback_stop_motion, &component_implementation_,_1,_2,component_config_));

        joint_states_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);
        state_ = n_.advertise<control_msgs::JointTrajectoryControllerState>("state", 1);
        diagnostics_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 1);
        robot_status_ = n_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);
        feedback_states_ = n_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
        command_vel_ = n_.subscribe("command_vel", 1, &canopen_402_node_ros::topicCallback_command_vel, this);
        command_pos_ = n_.subscribe("command_pos", 1, &canopen_402_node_ros::topicCallback_command_pos, this);
        joint_path_command_ = n_.subscribe("joint_path_command", 1, &canopen_402_node_ros::topicCallback_joint_path_command, this);
        joint_command_ = n_.subscribe("joint_command", 1, &canopen_402_node_ros::topicCallback_joint_command, this);

        np_.param("diagnostics_frequency", component_config_.diagnostics_frequency, (double)1.0);
        np_.param("chain_name", component_config_.chain_name, (std::string)"");
        np_.param("timeout", component_config_.timeout, (double)0.1);
        if(np_.hasParam("bus"))
            np_.getParam("bus", component_config_.bus);
        else
            ROS_ERROR("Parameter bus not set");
        if(np_.hasParam("modules"))
            np_.getParam("modules", component_config_.modules);
        else
            ROS_ERROR("Parameter modules not set");
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
    void topicCallback_joint_path_command(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
    {
        component_data_.in_joint_path_command = *msg;
    }
    void topicCallback_joint_command(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
    {
        component_data_.in_joint_command = *msg;
    }

    void configure_callback(ros_canopen_ros::canopen_402_nodeConfig &config, uint32_t level)
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
        component_data_.out_feedback_states_active = true;
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
        if (component_data_.out_feedback_states_active)
            feedback_states_.publish(component_data_.out_feedback_states);
    }
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "canopen_402_node");

    canopen_402_node_ros node;
    node.configure();

 // if cycle time == 0 do a spin() here without calling node.update()
    ros::spin();

    return 0;
}
