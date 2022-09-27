//   Copyright 2022 Christoph Hellmann Santos
//
//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.
//
#include <memory>
#include "canopen_base_driver/lely_driver_bridge.hpp"


const ros2_canopen::LelyBridgeErrCategory LelyBridgeErrCategoryInstance{};

std::error_code std::make_error_code(ros2_canopen::LelyBridgeErrc e)
{
  return {static_cast<int>(e), LelyBridgeErrCategoryInstance};
}

namespace ros2_canopen
{

  const char *LelyBridgeErrCategory::name() const noexcept
  {
    return "LelyBridgeError";
  }

  std::string LelyBridgeErrCategory::message(int ev) const
  {
    switch (static_cast<LelyBridgeErrc>(ev))
    {
    case LelyBridgeErrc::NotListedDevice:
      return "The CANopen device is not listed in object 1F81.";
    case LelyBridgeErrc::NoResponseOnDeviceType:
      return "No response received for upload request of object 1000.";
    case LelyBridgeErrc::DeviceTypeDifference:
      return "Value of object 1000 from CANopen device is different to value in object 1F84 (Device type).";
    case LelyBridgeErrc::VendorIdDifference:
      return "Value of object 1018:01 from CANopen device is different to value in object 1F85 (Vendor-ID).";
    case LelyBridgeErrc::HeartbeatIssue:
      return "Heartbeat event. No heartbeat message received from CANopen device.";
    case LelyBridgeErrc::NodeGuardingIssue:
      return "Node guarding event. No confirmation for guarding request received from CANopen device.";
    case LelyBridgeErrc::InconsistentProgramDownload:
      return "Objects for program download are not configured or inconsistent.";
    case LelyBridgeErrc::SoftwareUpdateRequired:
      return "Software update is required, but not allowed because of configuration or current status.";
    case LelyBridgeErrc::SoftwareDownloadFailed:
      return "Software update is required, but program download failed.";
    case LelyBridgeErrc::ConfigurationDownloadFailed:
      return "Configuration download failed.";
    case LelyBridgeErrc::StartErrorControlFailed:
      return "Heartbeat event during start error control service. No heartbeat message received from CANopen device during start error control service.";
    case LelyBridgeErrc::NmtSlaveInitiallyOperational:
      return "NMT slave was initially operational. (CANopen manager may resume operation with other CANopen devices)";
    case LelyBridgeErrc::ProductCodeDifference:
      return "Value of object 1018:02 from CANopen device is different to value in object 1F86 (Product code).";
    case LelyBridgeErrc::RevisionCodeDifference:
      return "Value of object 1018:03 from CANopen device is different to value in object 1F87 (Revision number).";
    case LelyBridgeErrc::SerialNumberDifference:
      return "Value of object 1018:04 from CANopen device is different to value in object 1F88 (Serial number).";
    default:
      return "(unrecognized error)";
    }
  }

  void LelyDriverBridge::OnState(canopen::NmtState state) noexcept
  {
    canopen::NmtState st = state;
    // We assume 1F80 bit 2 is false. All slaves are put into Operational after boot-up.
    // Lelycore does not track NMT states in this mode except BOOTUP.
    if (st == canopen::NmtState::BOOTUP)
    {
      st = canopen::NmtState::START;
    }

    if (!nmt_state_is_set.load())
    {
      // We do not care so much about missing a message, rather push them through.
      std::unique_lock<std::mutex> lk(nmt_mtex, std::defer_lock);
      if (lk.try_lock())
      {
        nmt_state_is_set.store(true);
        nmt_state_promise.set_value(st);
      }
    }
  }

  void LelyDriverBridge::OnBoot(canopen::NmtState st, char es, const ::std::string &what) noexcept
  {
    FiberDriver::OnBoot(st, es, what);
    if (es == 0)
    {
      booted.store(true);
    }
    std::unique_lock<std::mutex> lck(boot_mtex);
    this->boot_state = st;
    this->boot_status = es;
    this->boot_what = what;
    boot_cond.notify_all();
  }

  void LelyDriverBridge::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
  {
    uint32_t data = (uint32_t)rpdo_mapped[idx][subidx];
    COData codata = {idx, subidx, data, CODataTypes::CODataUnkown};

    // We do not care so much about missing a message, rather push them through.
    std::unique_lock<std::mutex> lk(pdo_mtex, std::defer_lock);
    if (lk.try_lock())
    {
      if (!rpdo_is_set.load())
      {
        rpdo_is_set.store(true);
        rpdo_promise.set_value(codata);
      }
    }
  }

  std::future<bool> LelyDriverBridge::async_sdo_write(COData data)
  {
    std::unique_lock<std::mutex> lck(sdo_mutex);
    if (running)
    {
      sdo_cond.wait(lck);
    }
    running = true;

    sdo_write_data_promise = std::make_shared<std::promise<bool>>();
    try
    {
      if (data.type_ == CODataTypes::COData8)
      {
        this->SubmitWrite(
            data.index_, data.subindex_, (uint8_t)data.data_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec) mutable
            {
              if (ec)
              {
                this->sdo_write_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id,
                        idx,
                        subidx,
                        ec,
                        "AsyncDownload"));
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
      else if (data.type_ == CODataTypes::COData16)
      {
        this->SubmitWrite(
            data.index_, data.subindex_, (uint16_t)data.data_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec) mutable
            {
              if (ec)
              {
                this->sdo_write_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id, idx, subidx, ec, "AsyncDownload"));
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
      else if (data.type_ == CODataTypes::COData32)
      {
        this->SubmitWrite(
            data.index_, data.subindex_, (uint32_t)data.data_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec) mutable
            {
              if (ec)
              {
                this->sdo_write_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id, idx, subidx, ec, "AsyncDownload"));
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
    }
    catch (lely::canopen::SdoError &e)
    {
      this->sdo_read_data_promise->set_exception(
          lely::canopen::make_sdo_exception_ptr(
              this->get_id(), data.index_, data.subindex_, e.code(), "AsyncUpload"));
      this->running = false;
      this->sdo_cond.notify_one();
    }

    return sdo_write_data_promise->get_future();
  }

  std::future<COData> LelyDriverBridge::async_sdo_read(COData data)
  {
    std::unique_lock<std::mutex> lck(sdo_mutex);
    if (running)
    {
      sdo_cond.wait(lck);
    }
    running = true;

    sdo_read_data_promise = std::make_shared<std::promise<COData>>();
    try
    {
      if (data.type_ == CODataTypes::COData8)
      {
        this->SubmitRead<uint8_t>(
            data.index_, data.subindex_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec, uint8_t value) mutable
            {
              if (ec)
              {
                this->sdo_read_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id, idx, subidx, ec, "AsyncUpload"));
              }
              else
              {
                COData d = {idx, subidx, value, CODataTypes::COData16};
                this->sdo_read_data_promise->set_value(d);
              }
              std::unique_lock<std::mutex> lck(this->sdo_mutex);
              this->running = false;
              this->sdo_cond.notify_one();
            },
            20ms);
      }
      else if (data.type_ == CODataTypes::COData16)
      {
        this->SubmitRead<uint16_t>(
            data.index_, data.subindex_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec, uint16_t value) mutable
            {
              if (ec)
              {
                this->sdo_read_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id, idx, subidx, ec, "AsyncUpload"));
              }
              else
              {
                COData d = {idx, subidx, value, CODataTypes::COData16};
                this->sdo_read_data_promise->set_value(d);
              }
              std::unique_lock<std::mutex> lck(this->sdo_mutex);
              this->running = false;
              this->sdo_cond.notify_one();
            },
            20ms);
      }
      else if (data.type_ == CODataTypes::COData32)
      {
        this->SubmitRead<uint16_t>(
            data.index_, data.subindex_,
            [this](uint8_t id, uint16_t idx, uint8_t subidx,
                   ::std::error_code ec, uint32_t value) mutable
            {
              if (ec)
              {
                this->sdo_read_data_promise->set_exception(
                    lely::canopen::make_sdo_exception_ptr(
                        id, idx, subidx, ec, "AsyncUpload"));
              }
              else
              {
                COData d = {idx, subidx, value, CODataTypes::COData16};
                this->sdo_read_data_promise->set_value(d);
              }
              std::unique_lock<std::mutex> lck(this->sdo_mutex);
              this->running = false;
              this->sdo_cond.notify_one();
            },
            20ms);
      }
    }
    catch (lely::canopen::SdoError &e)
    {
      this->sdo_read_data_promise->set_exception(
          lely::canopen::make_sdo_exception_ptr(
              this->get_id(), data.index_, data.subindex_, e.code(), "AsyncUpload"));
      this->running = false;
      this->sdo_cond.notify_one();
    }
    return sdo_read_data_promise->get_future();
  }

  std::future<canopen::NmtState> LelyDriverBridge::async_request_nmt()
  {
    std::scoped_lock<std::mutex> lk(nmt_mtex);
    nmt_state_is_set.store(false);
    nmt_state_promise = std::promise<canopen::NmtState>();
    return nmt_state_promise.get_future();
  }

  std::future<COData> LelyDriverBridge::async_request_rpdo()
  {
    std::scoped_lock<std::mutex> lk(pdo_mtex);
    rpdo_is_set.store(false);
    rpdo_promise = std::promise<COData>();
    return rpdo_promise.get_future();
  }

  void LelyDriverBridge::tpdo_transmit(COData data)
  {
    TPDOWriteTask task(this->GetStrand());
    task.driver = this;
    task.data = data;
    {
      this->master.GetExecutor().post(
          [&task, this]
          {
            this->GetStrand().post(task);
          });
    }
    // Wait for task to finish.
    std::scoped_lock<std::mutex> lk(task.mtx);
  }

  void LelyDriverBridge::nmt_command(canopen::NmtCommand command)
  {
    this->master.Command(command, nodeid);
  }

  uint8_t LelyDriverBridge::get_id()
  {
    return nodeid;
  }
}  // namespace ros2_canopen
