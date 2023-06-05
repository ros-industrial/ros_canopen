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

#include <sys/stat.h>

#include <lely/co/dcf.hpp>
#include <lely/co/dev.hpp>
#include <lely/co/obj.hpp>
#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/sdo_error.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/future.hpp>

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <boost/lockfree/queue.hpp>
#include <boost/optional.hpp>
#include <boost/thread.hpp>

#include "canopen_core/exchange.hpp"

using namespace std::chrono_literals;
using namespace lely;

namespace ros2_canopen
{
struct pdo_mapping
{
  bool is_tpdo;
  bool is_rpdo;
};

typedef std::map<uint16_t, std::map<uint8_t, pdo_mapping>> PDOMap;
class DriverDictionary : public lely::CODev
{
public:
  DriverDictionary(std::string eds_file) : lely::CODev(eds_file.c_str()) {}
  ~DriverDictionary()
  {
    // lely::CODev::~CODev();
  }

  std::shared_ptr<PDOMap> createPDOMapping()
  {
    std::shared_ptr<PDOMap> pdo_map = std::make_shared<PDOMap>();
    fetchRPDO(pdo_map);
    fetchTPDO(pdo_map);
    return pdo_map;
    // COObj * first = co_dev_first_obj((lely::CODev *)this);
    // COObj * last = co_dev_last_obj((lely::CODev *)this);
    // if (first == nullptr || last == nullptr)
    // {
    //   std::cout << "No objects found in dictionary" << std::endl;
    //   return pdo_map;
    // }
    // COObj * current = first;
    // while (current != last)
    // {
    //   uint16_t index = co_obj_get_idx(current);
    //   auto subids = current->getSubidx();
    //   bool created = false;
    //   for (auto subid : subids)
    //   {
    //     bool is_rpdo = checkObjRPDO(index, subid);
    //     bool is_tpdo = checkObjTPDO(index, subid);
    //     std::cout << "Found subobject: index=" << std::hex << (int)index
    //               << " subindex=" << (int)subid << (is_rpdo ? " rpdo" : "")
    //               << (is_tpdo ? " tpdo" : "") << std::endl;
    //     if (is_rpdo || is_tpdo)
    //     {
    //       pdo_mapping mapping;
    //       mapping.is_rpdo = is_rpdo;
    //       mapping.is_tpdo = is_tpdo;
    //       if (!created)
    //       {
    //         pdo_map->emplace(index, std::map<uint8_t, pdo_mapping>());
    //         created = true;
    //       }
    //       (*pdo_map)[index].emplace(subid, mapping);
    //     }
    //   }
    //   current = co_obj_next(current);
    // }
    // return pdo_map;
  }

  void fetchRPDO(std::shared_ptr<PDOMap> map)
  {
    for (int index = 0; index < 256; index++)
    {
      for (int subindex = 1; subindex < 9; subindex++)
      {
        auto obj = find(0x1600 + index, subindex);
        if (obj == nullptr)
        {
          continue;
        }
        uint32_t data;
        {
          data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
        }
        uint8_t tmps = (data >> 8) & 0xFF;
        uint16_t tmpi = (data >> 16) & 0xFFFF;
        if (tmpi == 0U)
        {
          continue;
        }
        pdo_mapping mapping;
        mapping.is_rpdo = true;
        mapping.is_tpdo = false;
        (*map)[tmpi][tmps] = mapping;
        std::cout << "Found rpdo mapped object: index=" << std::hex << (int)tmpi
                  << " subindex=" << (int)tmps << std::endl;
      }
    }
  }
  void fetchTPDO(std::shared_ptr<PDOMap> map)
  {
    for (int index = 0; index < 256; index++)
    {
      for (int subindex = 1; subindex < 9; subindex++)
      {
        auto obj = find(0x1A00 + index, subindex);
        if (obj == nullptr)
        {
          continue;
        }
        uint32_t data;
        {
          data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
        }
        uint8_t tmps = (data >> 8) & 0xFF;
        uint16_t tmpi = (data >> 16) & 0xFFFF;
        if (tmpi == 0U)
        {
          continue;
        }

        pdo_mapping mapping;
        mapping.is_rpdo = false;
        mapping.is_tpdo = true;
        (*map)[tmpi][tmps] = mapping;
        std::cout << "Found tpdo mapped object: index=" << std::hex << (int)tmpi
                  << " subindex=" << (int)tmps << std::endl;
      }
    }
  }

  bool checkObjRPDO(uint16_t idx, uint8_t subidx)
  {
    // std::cout << "Checking for rpo mapping of object: index=" << std::hex << (int)idx
    //           << " subindex=" << (int)subidx << std::endl;
    for (int i = 0; i < 256; i++)
    {
      if (this->checkObjInPDO(i, 0x1600, idx, subidx))
      {
        return true;
      }
    }
    return false;
  }

  bool checkObjTPDO(uint16_t idx, uint8_t subidx)
  {
    // std::cout << "Checking for rpo mapping of object: index=" << std::hex << (int)idx
    //          << " subindex=" << (int)subidx << std::endl;
    for (int i = 0; i < 256; i++)
    {
      if (this->checkObjInPDO(i, 0x1A00, idx, subidx))
      {
        return true;
      }
    }
    return false;
  }

  bool checkObjInPDO(uint8_t pdo, uint16_t mapping_idx, uint16_t idx, uint8_t subindex)
  {
    for (int i = 1; i < 9; i++)
    {
      auto obj = find(mapping_idx + pdo, i);
      if (obj == nullptr)
      {
        return false;
      }
      uint32_t data;
      {
        data = obj->getVal<CO_DEFTYPE_UNSIGNED32>();
      }
      uint8_t tmps = (data >> 8) & 0xFF;
      uint16_t tmpi = (data >> 16) & 0xFFFF;

      if (tmps == subindex && tmpi == idx)
      {
        std::cout << "Found object in pdo: " << (int)pdo << std::endl;
        return true;
      }
    }

    return false;
  }
};

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
  const char * name() const noexcept override;
  std::string message(int ev) const override;
};
}  // namespace ros2_canopen

namespace std
{
template <>
struct is_error_code_enum<ros2_canopen::LelyBridgeErrc> : true_type
{
};
std::error_code make_error_code(ros2_canopen::LelyBridgeErrc);
}  // namespace std

namespace ros2_canopen
{
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
protected:
  // Dictionary for driver based on DCF and BIN files.
  std::unique_ptr<DriverDictionary> dictionary_;
  std::mutex dictionary_mutex_;
  std::shared_ptr<PDOMap> pdo_map_;

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
  std::shared_ptr<SafeQueue<COData>> rpdo_queue;

  // EMCY synchronisation items
  std::promise<COEmcy> emcy_promise;
  std::atomic<bool> emcy_is_set;
  std::mutex emcy_mtex;
  std::shared_ptr<SafeQueue<COEmcy>> emcy_queue;

  // BOOT synchronisation items
  std::atomic<bool> booted;
  char boot_status;
  std::string boot_what;
  canopen::NmtState boot_state;
  std::condition_variable boot_cond;
  std::mutex boot_mtex;

  uint8_t nodeid;
  std::string name_;

  std::function<void()> on_sync_function_;

  // void set_sync_function(std::function<void()> on_sync_function)
  // {
  //   on_sync_function_ = on_sync_function;
  // }

  // void unset_sync_function()
  // {
  //   on_sync_function_ = std::function<void()>();
  // }

  void OnSync(uint8_t cnt, const time_point & t) noexcept override
  {
    if (on_sync_function_ != nullptr)
    {
      try
      {
        on_sync_function_();
      }
      catch (...)
      {
      }
    }
  }

  /**
   * @brief OnState Callback
   *
   * This callback function is called when an Nmt state
   * change is detected on the connected device.
   *
   * @param [in] state    NMT State
   */
  void OnState(canopen::NmtState state) noexcept override;

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
  virtual void OnBoot(canopen::NmtState st, char es, const ::std::string & what) noexcept override;

  /**
   * @brief OnRpdoWrite Callback
   *
   * This callback function is called when an RPDO
   * write request is received from the connected device.
   * @todo This function should use a threadsafe queue not the icky implementation we have now.
   *
   * @param [in] idx      Object Index
   * @param [in] subidx   Object Subindex
   */
  void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override;

  /**
   * The function invoked when an EMCY message is received from the remote node.
   * @todo This function should use a threadsafe queue not the icky implementation we have now.
   *
   * @param eec  the emergency error code.
   * @param er   the error register.
   * @param msef the manufacturer-specific error code.
   */
  void OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept override;

public:
  using FiberDriver::FiberDriver;

  /**
   * @brief Construct a new Lely Bridge object
   *
   * @param [in] exec     Executor to use
   * @param [in] master   Master to use
   * @param [in] id       NodeId to connect to
   * @param [in] eds      EDS file
   * @param [in] bin      BIN file (concise dcf)
   *
   */
  LelyDriverBridge(
    ev_exec_t * exec, canopen::AsyncMaster & master, uint8_t id, std::string name, std::string eds,
    std::string bin)
  : FiberDriver(exec, master, id),
    rpdo_queue(new SafeQueue<COData>()),
    emcy_queue(new SafeQueue<COEmcy>())
  {
    nodeid = id;
    running = false;
    name_ = name;
    dictionary_ = std::make_unique<DriverDictionary>(eds.c_str());
    struct stat buffer;
    if (stat(bin.c_str(), &buffer) == 0)
    {
      co_unsigned16_t * a = NULL;
      co_unsigned16_t * b = NULL;
      dictionary_->readDCF(a, b, bin.c_str());
    }
    pdo_map_ = dictionary_->createPDOMapping();
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
   * stored when the write request was unsuccessful.
   */
  std::future<bool> async_sdo_write(COData data);

  template <typename T>
  std::future<bool> async_sdo_write_typed(uint16_t idx, uint8_t subidx, T value)
  {
    std::unique_lock<std::mutex> lck(sdo_mutex);
    if (running)
    {
      sdo_cond.wait(lck);
    }
    running = true;

    auto prom = std::make_shared<std::promise<bool>>();
    lely::COSub * sub = this->dictionary_->find(idx, subidx);
    if (sub == nullptr)
    {
      std::cout << "async_sdo_write_typed: id=" << (unsigned int)this->get_id() << " index=0x"
                << std::hex << (unsigned int)idx << " subindex=" << (unsigned int)subidx
                << " object does not exist" << std::endl;
      prom->set_value(false);
      this->running = false;
      this->sdo_cond.notify_one();
      return prom->get_future();
    }

    this->SubmitWrite(
      idx, subidx, value,
      [this, value, prom](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) mutable
      {
        if (ec)
        {
          prom->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncDownload"));
        }
        else
        {
          std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
          this->dictionary_->setVal<T>(idx, subidx, value);
          prom->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
    return prom->get_future();
  }

  template <typename T>
  bool sync_sdo_write_typed(
    uint16_t idx, uint8_t subidx, T value, std::chrono::milliseconds timeout)
  {
    auto fut = async_sdo_write_typed(idx, subidx, value);
    auto wait_res = fut.wait_for(timeout);
    if (wait_res == std::future_status::timeout)
    {
      std::cout << "sync_sdo_write_typed: id=" << (unsigned int)this->get_id() << " index=0x"
                << std::hex << (unsigned int)idx << " subindex=" << (unsigned int)subidx
                << " timed out." << std::endl;
      return false;
    }
    bool res = false;
    try
    {
      res = fut.get();
    }
    catch (std::exception & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
    }
    return res;
  }

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
   * request was unsuccessful.
   */
  std::future<COData> async_sdo_read(COData data);

  template <typename T>
  std::future<T> async_sdo_read_typed(uint16_t idx, uint8_t subidx)
  {
    std::unique_lock<std::mutex> lck(sdo_mutex);
    if (running)
    {
      sdo_cond.wait(lck);
    }
    running = true;

    auto prom = std::make_shared<std::promise<T>>();
    lely::COSub * sub = this->dictionary_->find(idx, subidx);
    if (sub == nullptr)
    {
      std::cout << "async_sdo_read: id=" << (unsigned int)this->get_id() << " index=0x" << std::hex
                << (unsigned int)idx << " subindex=" << (unsigned int)subidx
                << " object does not exist" << std::endl;
      try
      {
        throw lely::canopen::SdoError(this->get_id(), idx, subidx, lely::canopen::SdoErrc::NO_OBJ);
      }
      catch (...)
      {
        prom->set_exception(std::current_exception());
      }
      this->running = false;
      this->sdo_cond.notify_one();
      return prom->get_future();
    }
    this->SubmitRead<T>(
      idx, subidx,
      [this, prom](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, T value) mutable
      {
        if (ec)
        {
          prom->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncUpload"));
        }
        else
        {
          std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
          this->dictionary_->setVal<T>(idx, subidx, value);
          prom->set_value(value);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
    return prom->get_future();
  }

  template <typename T>
  bool sync_sdo_read_typed(
    uint16_t idx, uint8_t subidx, T & value, std::chrono::milliseconds timeout)
  {
    auto fut = async_sdo_read_typed<T>(idx, subidx);
    auto wait_res = fut.wait_for(timeout);
    if (wait_res == std::future_status::timeout)
    {
      std::cout << "sync_sdo_read_typed: id=" << (unsigned int)this->get_id() << " index=0x"
                << std::hex << (unsigned int)idx << " subindex=" << (unsigned int)subidx
                << " timed out." << std::endl;
      return false;
    }
    bool res = false;
    try
    {
      value = fut.get();
      res = true;
    }
    catch (std::exception & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
      res = false;
    }
    return res;
  }

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
   * @todo This function should use a threadsafe queue not the icky implementation we have now.
   *
   * @return std::future<COData>
   * The returned future is set when an RPDO event is detected.
   */
  std::shared_ptr<SafeQueue<COData>> get_rpdo_queue();

  /**
   * @brief Asynchronous request for EMCY
   * @todo This function should use a threadsafe queue not the icky implementation we have now.
   *
   * @return std::future<COEmcy>
   * The returned future is set when an EMCY event is detected.
   */
  std::shared_ptr<SafeQueue<COEmcy>> get_emcy_queue();

  /**
   * @brief Executes a TPDO transmission
   *
   * This function executes a TPDO transmission. The{false, true}
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
      return true;
    }
    return false;
  }

  void set_sync_function(std::function<void()> on_sync_function)
  {
    on_sync_function_ = on_sync_function;
  }

  void unset_sync_function() { on_sync_function_ = std::function<void()>(); }

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
  bool is_booted() { return booted.load(); }

  template <typename T>
  void submit_write(COData data)
  {
    T value = 0;
    std::memcpy(&value, &data.data_, sizeof(value));

    this->SubmitWrite(
      data.index_, data.subindex_, value,
      [this, value](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec) mutable
      {
        if (ec)
        {
          this->sdo_write_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncDownload"));
        }
        else
        {
          std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
          this->dictionary_->setVal<T>(idx, subidx, value);
          this->sdo_write_data_promise->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }

  template <typename T>
  void submit_read(COData data)
  {
    this->SubmitRead<T>(
      data.index_, data.subindex_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx, ::std::error_code ec, T value) mutable
      {
        if (ec)
        {
          this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(id, idx, subidx, ec, "AsyncUpload"));
        }
        else
        {
          std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
          this->dictionary_->setVal<T>(idx, subidx, value);
          COData d = {idx, subidx, 0};
          std::memcpy(&d.data_, &value, sizeof(T));
          this->sdo_read_data_promise->set_value(d);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }

  template <typename T>
  const T universal_get_value(uint16_t index, uint8_t subindex)
  {
    T value = 0;
    bool is_tpdo = false;
    if (this->pdo_map_->find(index) != this->pdo_map_->end())
    {
      auto object = this->pdo_map_->at(index);
      if (object.find(subindex) != object.end())
      {
        auto entry = object.at(subindex);
        is_tpdo = entry.is_tpdo;
      }
    }
    if (!is_tpdo)
    {
      if (sync_sdo_read_typed<T>(index, subindex, value, std::chrono::milliseconds(20)))
      {
        return value;
      }
    }

    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    if (typeid(T) == typeid(uint8_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED8>(index, subindex);
    }
    if (typeid(T) == typeid(uint16_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED16>(index, subindex);
    }
    if (typeid(T) == typeid(uint32_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_UNSIGNED32>(index, subindex);
    }
    if (typeid(T) == typeid(int8_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_INTEGER8>(index, subindex);
    }
    if (typeid(T) == typeid(int16_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_INTEGER16>(index, subindex);
    }
    if (typeid(T) == typeid(int32_t))
    {
      value = this->dictionary_->getVal<CO_DEFTYPE_INTEGER32>(index, subindex);
    }

    return value;
  }

  template <typename T>
  void universal_set_value(uint16_t index, uint8_t subindex, T value)
  {
    bool is_rpdo = false;
    if (this->pdo_map_->find(index) != this->pdo_map_->end())
    {
      auto object = this->pdo_map_->at(index);
      if (object.find(subindex) != object.end())
      {
        auto entry = object.at(subindex);
        is_rpdo = entry.is_rpdo;
      }
    }
    if (is_rpdo)
    {
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      this->dictionary_->setVal<T>(index, subindex, value);
      this->tpdo_mapped[index][subindex] = value;
      this->tpdo_mapped[index][subindex].WriteEvent();
    }
    else
    {
      sync_sdo_write_typed(index, subindex, value, std::chrono::milliseconds(20));
    }
  }
};

}  // namespace ros2_canopen

#endif  // CANOPEN_BASE_DRIVER__LELY_BRIDGE_HPP_
