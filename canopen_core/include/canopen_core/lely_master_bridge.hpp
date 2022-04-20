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
     * @brief Creates services to pass values between Lely and ROS executor.
     *
     */
    class LelyMasterBridge : public lely::canopen::AsyncMaster
    {

        std::shared_ptr<std::promise<COData>> sdo_read_data_promise;
        std::shared_ptr<std::promise<bool>> sdo_write_data_promise;
        std::promise<bool> nmt_promise;
        std::mutex sdo_mutex;
        bool running;
        std::condition_variable sdo_cond;

    public:
        LelyMasterBridge(
            ev_exec_t *exec,
            lely::io::TimerBase &timer,
            lely::io::CanChannelBase &chan,
            const std::string &dcf_txt,
            const std::string &dcf_bin = "",
            uint8_t id = (uint8_t)255U) : lely::canopen::AsyncMaster(exec, timer, chan, dcf_txt, dcf_bin, id)
        {
        }

        std::future<bool> async_write_sdo(uint8_t id, COData data);
        std::future<COData> async_read_sdo(uint8_t id, COData data);

        /**
         * @brief async_write_nmt
         * 
         * @param id 
         * @param command 
         * @return std::future<bool> 
         * 
         * @todo Catch errors and transmit via future
         */
        std::future<bool> async_write_nmt(uint8_t id, uint8_t command);
    };

}  // ros2_canopen

#endif // LELY_MASTER_BRIDGE_HPP