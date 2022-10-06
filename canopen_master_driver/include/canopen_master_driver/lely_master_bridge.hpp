//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
// 
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef LELY_MASTER_BRIDGE_HPP
#define LELY_MASTER_BRIDGE_HPP

#include "lely/coapp/master.hpp"
#include <lely/ev/loop.hpp>
#include <lely/io2/posix/poll.hpp>

#include <memory>
#include <mutex>
#include <future>
#include <condition_variable>
#include "canopen_core/exchange.hpp"

namespace ros2_canopen
{
    /**
     * @brief Lely Master Bridge
     * 
     * Creates services to pass values between the CANopen executor and ROS executor.
     * This is important because certain functions can only be called from within
     * the thread of the CANopen executor.
     * 
     */
    class LelyMasterBridge : public lely::canopen::AsyncMaster
    {

        std::shared_ptr<std::promise<COData>> sdo_read_data_promise;        ///< Pointer to Promise for read service calls.
        std::shared_ptr<std::promise<bool>> sdo_write_data_promise;         ///< Pointer to Promise for write service calls
        std::promise<bool> nmt_promise;                                     ///< Pointer to Promise for nmt service calls 
        std::mutex sdo_mutex;                                               ///< Mutex to guard sdo objects
        bool running;                                                       ///< Bool to indicate whether an sdo call is running
        std::condition_variable sdo_cond;                                   ///< Condition variable to sync service calls (one at a time)

    public:

        /**
         * @brief Construct a new Lely Master Bridge object
         * 
         * @param [in] exec         Pointer to the executor
         * @param [in] timer        Pointer to the timer
         * @param [in] chan         Pointer to the channel
         * @param [in] dcf_txt      Path to the DCF file
         * @param [in] dcf_bin      Path to the DCF bin file
         * @param [in] id           CANopen node id of the master
         */
        LelyMasterBridge(
            ev_exec_t *exec,
            lely::io::TimerBase &timer,
            lely::io::CanChannelBase &chan,
            const std::string &dcf_txt,
            const std::string &dcf_bin = "",
            uint8_t id = (uint8_t)255U) : lely::canopen::AsyncMaster(exec, timer, chan, dcf_txt, dcf_bin, id)
        {
        }

        /**
         * @brief Asynchronous call for writing to an SDO
         * 
         * @param [in] id               CANopen node id of the target device
         * @param [in] data             Data to write
         * @return std::future<bool>    A future that indicates wether the
         * write Operation was successful.
         */
        std::future<bool> async_write_sdo(uint8_t id, COData data);

        /**
         * @brief 
         * 
         * @param [in] id               CANopen node id of the target device
         * @param [in] data             Data to read, value is disregarded
         * @return std::future<COData>  A future that returns the read result
         * once the process finished.
         */
        std::future<COData> async_read_sdo(uint8_t id, COData data);

        /**
         * @brief async_write_nmt
         * 
         * @param id 
         * @param command 
         * @return std::future<bool> 
         * 
         */
        std::future<bool> async_write_nmt(uint8_t id, uint8_t command);
    };

}  // ros2_canopen

#endif // LELY_MASTER_BRIDGE_HPP