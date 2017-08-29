/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/socketcan_recover_ctrl.h>


namespace socketcan_bridge
{
    
    /**
     * @brief Constructor for the recovery control class
     *
     * Initializes the publisher, and sets up a timer to periodically check the bus state
     */
    SocketCANRecoverCtrl::SocketCANRecoverCtrl(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
        boost::shared_ptr<can::DriverInterface> driver)
    {
        state_pub_ = nh->advertise<can_msgs::CanState>("can_state", 1000);
        driver_ = driver;
        timeout_ = ros::Duration(1); // will be reconfigured
        timer_ = nh->createWallTimer(ros::WallDuration(1), [this](const ros::WallTimerEvent& event) {CheckState();});
        //dynamic reconfig
        config_server_.setCallback([this](socketcan_bridge::SocketCANConfig& cfg, uint32_t level) { dynReconfigCallback(cfg, level); });

    };

    /**
     * @brief Checks the state of the bus, recover if necessary
     */
    void SocketCANRecoverCtrl::CheckState() {
        curr_state_ = driver_->getState();
        publishStatus();
        // Last_error_time denotes the beginning of a fault
        // clear it when the bus is working properly
        if(curr_state_.isReady()) {
            last_error_time_.sec = 0;
            last_error_time_.nsec = 0;
        }
        // if this is the first error, set the time
        else if (last_error_time_.sec == 0 && last_error_time_.nsec == 0) {
            last_error_time_ = ros::Time::now();
        }
        // if this isn't the first consecutive error, check how long
        // the error state has lasted and recover if necessary
        else {
            ros::Duration error_duration = ros::Time::now() - last_error_time_;
            if(error_duration > timeout_) {
                recover();
            }
        }
        
    }

    /**
     * @brief Recover the bus from an error state
     *
     * Calls the driver's recover() function
     */
    void SocketCANRecoverCtrl::recover() {
        // Optimistically assume that it will work and reset timeout - 
        // if it fails, it'll time out again later
        last_error_time_.sec = 0;
        last_error_time_.nsec = 0;
        bool success = false;
        if(mutex_.try_lock()) {
            success = driver_->recover();
            if(success) {
                ROS_INFO("CAN driver timed out, successfully recovered");
            } else {
                ROS_WARN("CAN driver timed out, recovery failed");
            }
            mutex_.unlock();
        }
        
    }

    /**
     * @brief Publishes the status of the bus
     */
    void SocketCANRecoverCtrl::publishStatus() {
        static unsigned int seq = 0;

        can_msgs::CanState state_msg;

        switch(curr_state_.driver_state) {
            case can::State::open:
                state_msg.driver_state = can_msgs::CanState::OPEN;
                break;
            case can::State::closed:
                state_msg.driver_state = can_msgs::CanState::CLOSED;
                break;
            case can::State::ready:
                state_msg.driver_state = can_msgs::CanState::READY;
                break;
            default:
                state_msg.driver_state = can_msgs::CanState::CLOSED;
                break;
        }

        state_pub_.publish(state_msg);
    }

    void SocketCANRecoverCtrl::dynReconfigCallback(socketcan_bridge::SocketCANConfig &config, uint32_t level) {
        timeout_ = ros::Duration(config.timeout);
        ROS_INFO("Reconfigured timeout to %d seconds", config.timeout);
    }


};  // namespace socketcan_bridge
