#ifndef LELY_MASTER_BRIDGE_HPP
#define LELY_MASTER_BRIDGE_HPP

#include "lely/coapp/master.hpp"

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