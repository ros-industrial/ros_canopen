//    Copyright 2022 Christoph Hellmann Santos
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

#ifndef BASIC_DEVICE_DRIVER_HPP
#define BASIC_DEVICE_DRIVER_HPP
#include <memory>
#include <mutex>
#include <atomic>
#include <future>
#include <thread>
#include <condition_variable>

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/future.hpp>
#include <lely/ev/co_task.hpp>

using namespace std::chrono_literals;
using namespace lely;

namespace ros2_canopen
{
        enum CODataTypes
    {
        CODataUnkown = 0,
        COData8 = 8,
        COData16 = 16,
        COData32 = 32
    };
    //template<typename T>
    struct COData
    {
    public:
        uint16_t index_;
        uint8_t subindex_;
        uint32_t data_;
        CODataTypes type_;
    };

    /**
     * @brief Basic CANopen Device Driver
     * 
     * This class provides a basic CANopen Device Driver that
     * can communicate with ROS2 via asynchronous functions.
     * 
     */
    class BasicDeviceDriver : public canopen::FiberDriver
    {
        class TPDOWriteTask : public ev::CoTask
        {
        public:
            COData data;
            BasicDeviceDriver *driver;
            std::mutex mtx;
            TPDOWriteTask(ev_exec_t *exec) : ev::CoTask(exec)
            {
                // Lock and load
                mtx.lock();
            }
            void operator()() noexcept
            {
                std::scoped_lock<util::BasicLockable> lk(driver->tpdo_event_mutex);
                switch (data.type_)
                {
                case CODataTypes::COData8:
                    driver->tpdo_mapped[data.index_][data.subindex_] = static_cast<uint8_t>(data.data_);
                    break;
                case CODataTypes::COData16:
                    driver->tpdo_mapped[data.index_][data.subindex_] = static_cast<uint16_t>(data.data_);
                    break;
                case CODataTypes::COData32:
                    driver->tpdo_mapped[data.index_][data.subindex_] = static_cast<uint32_t>(data.data_);
                    break;
                default:
                    break;
                }
                driver->master.TpdoWriteEvent(driver->id(), data.index_, data.subindex_);
                // Unlock when done
                mtx.unlock();
            }
        };

    private:

        // SDO Read synchronisation items
        std::shared_ptr<std::promise<COData>> sdo_read_data_promise;
        std::shared_ptr<std::promise<bool>> sdo_write_data_promise;
        std::mutex sdo_mutex;
        bool running;
        std::condition_variable sdo_cond;

        // NMT synchronisation items
        std::promise<canopen::NmtState> nmt_state_promise;
        std::atomic<bool> nmt_state_is_set;
        std::mutex nmt_mtex;

        // RPDO synchronisation items
        std::promise<COData> rpdo_promise;
        std::atomic<bool> rpdo_is_set;
        std::mutex pdo_mtex;


        std::vector<std::shared_ptr<TPDOWriteTask>> tpdo_tasks;


        uint8_t nodeid;



        void
        OnState(canopen::NmtState state) noexcept override;

        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;


    public:
        using FiberDriver::FiberDriver;
        BasicDeviceDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id)
            : FiberDriver(exec, master, id)
        {
            nodeid = id;
            running = false;
        }

        /**
         * @brief Asynchronous SDO Write
         * 
         * @param data 
         * @return std::future<bool> 
         */
        std::future<bool> async_sdo_write(COData data);
        /**
         * @brief Aynchronous SDO Read
         * 
         * @param data 
         * @return std::future<COData> 
         */
        std::future<COData> async_sdo_read(COData data);

        /**
         * @brief Asynchronous request for NMT
         * 
         * @return std::future<canopen::NmtState> 
         * The returned future is set when NMT State changes.
         */
        std::future<canopen::NmtState> async_request_nmt();

        /**
         * @brief Asynchronous request for RPDO
         * 
         * @return std::future<COData> 
         * The returned future is set when an RPDO event is detected.
         */
        std::future<COData> async_request_rpdo(); 

        /**
         * @brief Executes a TPDO transmission
         * 
         * @param data 
         */
        void tpdo_transmit(COData data);

        /**
         * @brief Executes a NMT Command
         * 
         * @param command 
         */
        void nmt_command(canopen::NmtCommand command);

        /**
         * @brief Get the nodeid
         * 
         * @return uint8_t 
         */
        uint8_t get_id();
    };

}



#endif
