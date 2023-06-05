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

#include <lely/ev/loop.hpp>
#include <lely/io2/posix/poll.hpp>
#include "lely/coapp/master.hpp"

#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include "canopen_core/exchange.hpp"

using namespace std::literals::chrono_literals;
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
  std::shared_ptr<std::promise<COData>>
    sdo_read_data_promise;  ///< Pointer to Promise for read service calls.
  std::shared_ptr<std::promise<bool>>
    sdo_write_data_promise;          ///< Pointer to Promise for write service calls
  std::promise<bool> nmt_promise;    ///< Pointer to Promise for nmt service calls
  std::mutex sdo_mutex;              ///< Mutex to guard sdo objects
  bool running;                      ///< Bool to indicate whether an sdo call is running
  std::condition_variable sdo_cond;  ///< Condition variable to sync service calls (one at a time)
  uint8_t node_id;                   ///< Node id of the master

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
    ev_exec_t * exec, lely::io::TimerBase & timer, lely::io::CanChannelBase & chan,
    const std::string & dcf_txt, const std::string & dcf_bin = "", uint8_t id = (uint8_t)255U)
  : lely::canopen::AsyncMaster(exec, timer, chan, dcf_txt, dcf_bin, id), node_id(id)
  {
  }

  /**
   * @brief Asynchronous call for writing to an SDO
   *
   * @param [in] id               CANopen node id of the target device
   * @param [in] data             Data to write
   * @param [in] datatype         Datatype of the data to write
   * @return std::future<bool>    A future that indicates if the
   * write Operation was successful.
   */
  std::future<bool> async_write_sdo(uint8_t id, COData data, uint8_t datatype);

  /**
   * @brief
   *
   * @param [in] id               CANopen node id of the target device
   * @param [in] data             Data to read, value is disregarded
   * @param [in] datatype         Datatype of the data to read
   * @return std::future<COData>  A future that returns the read result
   * once the process finished.
   */
  std::future<COData> async_read_sdo(uint8_t id, COData data, uint8_t datatype);

  /**
   * @brief async_write_nmt
   *
   * @param id
   * @param command
   * @return std::future<bool>
   *
   */
  std::future<bool> async_write_nmt(uint8_t id, uint8_t command);

  template <typename T>
  void submit_write_sdo(uint8_t id, uint16_t idx, uint8_t subidx, T value)
  {
    this->SubmitWrite(
      this->GetExecutor(), id, idx, subidx, value,
      [this](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) mutable
      {
        if (ec)
        {
          this->sdo_write_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncDownload"));
        }
        else
        {
          this->sdo_write_data_promise->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }

  template <typename T>
  void submit_read_sdo(uint8_t id, uint16_t idx, uint8_t subidx)
  {
    this->SubmitRead<T>(
      this->GetExecutor(), id, idx, subidx,
      [this](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, T value) mutable
      {
        if (ec)
        {
          this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncUpload"));
        }
        else
        {
          COData codata = {idx, subidx, 0U};
          std::memcpy(&codata.data_, &value, sizeof(T));
          this->sdo_read_data_promise->set_value(codata);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }
};

}  // namespace ros2_canopen

#endif  // LELY_MASTER_BRIDGE_HPP
