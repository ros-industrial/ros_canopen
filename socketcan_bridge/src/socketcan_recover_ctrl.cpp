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
        boost::shared_ptr<can::DriverInterface> driver) : driver_(driver)
    {
        state_pub_ = nh->advertise<can_msgs::CanState>("can_state", 1, true);

        timer_ = nh->createWallTimer(ros::WallDuration(5), [this](const ros::WallTimerEvent& event) {recover();}, true, false);

        state_listener_ = driver_->createStateListener(
            can::StateInterface::StateDelegate(this, &SocketCANRecoverCtrl::stateCallback));

    }

    /**
     * @brief Checks the state of the bus, if !statie.isReady() then the
     * recover timer is started to fire in 5secs, otherwise we stop the timer
     */
    void SocketCANRecoverCtrl::stateCallback(const can::State & state) {
        publishStatus(state);
        if(!state.isReady())
        {            
            timer_.start();
        }
        else
        {
            timer_.stop();
        }
    }

    /**
     * @brief Recover the bus from an error state
     *
     * Calls the driver's recover() function
     */
    void SocketCANRecoverCtrl::recover() 
    {
        timer_.stop();
        if(driver_->recover()) {
            ROS_INFO("CAN driver timed out, successfully recovered");
        } 
        else 
        {
            ROS_WARN("CAN driver timed out, recovery failed");
            timer_.start();
        }
    }

    /**
     * @brief Publishes the status of the bus
     */
    void SocketCANRecoverCtrl::publishStatus(const can::State & state) {
        can_msgs::CanState state_msg;
        switch(state.driver_state) {
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


};  // namespace socketcan_bridge
