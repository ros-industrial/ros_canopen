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

#ifndef CANOPEN_BASE_DRIVER__LELY_BRIDGE_HPP_
#define CANOPEN_BASE_DRIVER__LELY_BRIDGE_HPP_

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/ev/future.hpp>
#include <lely/ev/co_task.hpp>

#include <memory>
#include <mutex>
#include <atomic>
#include <future>
#include <thread>
#include <vector>
#include <condition_variable>
#include <system_error>
#include <rclcpp/rclcpp.hpp>

#include "canopen_core/exchange.hpp"

using namespace std::chrono_literals;
using namespace lely;

namespace ros2_canopen
{

  enum class LelyBridgeErrc
  {
    NotListedDevice = 'A',
    NoResponseOnDeviceType = 'B',
    DeviceTypeDifference = 'C',
    VendorIdDifference = 'D',
    HeartbeatIssue = 'E',
    NodeGuardingIssue = 'F',
    InconsistentProgramDownload = 'G',
    SoftwareUpdateRequired = 'H',
    SoftwareDownloadFailed = 'I',
    ConfigurationDownloadFailed = 'J',
    StartErrorControlFailed = 'K',
    NmtSlaveInitiallyOperational = 'L',
    ProductCodeDifference = 'M',
    RevisionCodeDifference = 'N',
    SerialNumberDifference = 'O'
  };

  struct LelyBridgeErrCategory : std::error_category
  {
    const char *name() const noexcept override;
    std::string message(int ev) const override;
  };
}

namespace std
{
  template <>
  struct is_error_code_enum<ros2_canopen::LelyBridgeErrc> : true_type
  {
  };
  std::error_code make_error_code(ros2_canopen::LelyBridgeErrc);
}

namespace ros2_canopen{
  /**
   * @brief Lely Driver Bridge
   *
   * This class provides functionalities for bridging between
   * Lelycore drivers and standard C++ functions. This means
   * it provides async and sync functions for interacting with
   * CANopen devices using synchronisation functionalities from C++ standard
   * library.
   *
   */
  class LelyDriverBridge : public canopen::FiberDriver
  {
    class TPDOWriteTask : public ev::CoTask
    {
    public:
      COData data;
      LelyDriverBridge *driver;
      std::mutex mtx;
      explicit TPDOWriteTask(ev_exec_t *exec)
          : ev::CoTask(exec)
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
          driver->tpdo_mapped[data.index_][data.subindex_] =
              static_cast<uint8_t>(data.data_);
          break;
        case CODataTypes::COData16:
          driver->tpdo_mapped[data.index_][data.subindex_] =
              static_cast<uint16_t>(data.data_);
          break;
        case CODataTypes::COData32:
          driver->tpdo_mapped[data.index_][data.subindex_] =
              static_cast<uint32_t>(data.data_);
          break;
        default:
          break;
        }
        driver->master.TpdoWriteEvent(driver->id(), data.index_, data.subindex_);
        // Unlock when done
        mtx.unlock();
      }
    };

  protected:
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

    // BOOT synchronisation items
    std::atomic<bool> booted;
    char boot_status;
    std::string boot_what;
    canopen::NmtState boot_state;
    std::condition_variable boot_cond;
    std::mutex boot_mtex;

    std::vector<std::shared_ptr<TPDOWriteTask>> tpdo_tasks;
    uint8_t nodeid;
    std::string name_;

    /**
     * @brief OnState Callback
     *
     * This callback function is called when an Nmt state
     * change is detected on the connected device.
     *
     * @param [in] state    NMT State
     */
    void
    OnState(canopen::NmtState state) noexcept override;

    /**
     * @brief OnBoot Callback
     * This callback is called when the Boot process of the 
     * slave that was initiated by the master has been success
     * fully finished.
     *
     * @param st
     * @param es
     * @param what
     */
    virtual void
    OnBoot(canopen::NmtState st, char es, const ::std::string &what) noexcept override;

    /**
     * @brief OnRpdoWrite Callback
     *
     * This callback function is called when an RPDO
     * write request is received from the connected device.
     *
     * @param [in] idx      Object Index
     * @param [in] subidx   Object Subindex
     */
    void
    OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  public:
    using FiberDriver::FiberDriver;

    /**
     * @brief Construct a new Lely Bridge object
     *
     * @param [in] exec     Executor to use
     * @param [in] master   Master to use
     * @param [in] id       NodeId to connect to
     */
    LelyDriverBridge(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id, std::string name)
        : FiberDriver(exec, master, id)
    {
      nodeid = id;
      running = false;
      name_ = name;

    }

    /**
     * @brief Asynchronous SDO Write
     *
     * Writes the data passed to the function via SDO to
     * the connected device.
     *
     * @param [in] data     Data to written.
     *
     * @return std::future<bool>
     * Returns an std::future<bool> that is fulfilled
     * when the write request was done. An error is
     * stored when the write request was unsuccesful.
     */
    std::future<bool> async_sdo_write(COData data);

    /**
     * @brief Aynchronous SDO Read
     *
     * Reads the indicated SDO object from the connected
     * device.
     *
     * @param [in] data       Data to be read, the data entry is not used.
     * @return std::future<COData>
     * Returns an std::future<COData> that is fulfilled
     * when the read request was done. The result of the request
     * is stored in the future. An error is stored when the read
     * request was unsuccesful.
     */
    std::future<COData> async_sdo_read(COData data);

    /**
     * @brief Asynchronous request for NMT
     *
     * Waits for an NMT state change to occur. The new
     * state is stored in the future returned by the function.
     *
     * @return std::future<canopen::NmtState>
     * The returned future is set when NMT State changes.
     */
    std::future<canopen::NmtState> async_request_nmt();

    /**
     * @brief Asynchronous request for RPDO
     *
     * Waits for an RPDO write request to be received from
     * the slave. The content of the request are stored in
     * the returned future.
     *
     * @return std::future<COData>
     * The returned future is set when an RPDO event is detected.
     */
    std::future<COData> async_request_rpdo();

    /**
     * @brief Executes a TPDO transmission
     *
     * This funciton executes a TPDO transmission. The
     * object specified in the input data is sent if it
     * is registered as a TPDO with the master.
     *
     * @param [in] data       Object and data to be written
     */
    void tpdo_transmit(COData data);

    /**
     * @brief Executes a NMT Command
     *
     * This function sends the NMT command specified as
     * parameter.
     *
     * @param [in] command    NMT Command to execute
     */
    void nmt_command(canopen::NmtCommand command);

    /**
     * @brief Get the nodeid
     *
     * @return uint8_t
     */
    uint8_t get_id();


    /**
     * @brief Wait for device to be booted
     * 
     * @return true 
     * @return false 
     */
    bool wait_for_boot()
    {
      if (booted.load())
      {
        return true;
      }
      std::unique_lock<std::mutex> lck(boot_mtex);
      boot_cond.wait(lck);
      if ((boot_status != 0) && (boot_status != 'L'))
      {
        throw std::system_error(boot_status, LelyBridgeErrCategory(), "Boot Issue");
      }
      else
      {
        booted.store(true);
      }
    }

    /**
     * @brief Request master to boot device
     * 
     */
    void Boot()
    {
      booted.store(false);
      FiberDriver::Boot();
    }

    /**
     * @brief Indicates if Device is booted
     * 
     * @return true 
     * @return false 
     */
    bool is_booted()
    {
      return booted.load();
    }
  };

} // namespace ros2_canopen

#endif // CANOPEN_BASE_DRIVER__LELY_BRIDGE_HPP_
