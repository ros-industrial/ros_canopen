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
#include "canopen_base_driver/lely_driver_bridge.hpp"
#include <bitset>
#include <memory>

const ros2_canopen::LelyBridgeErrCategory LelyBridgeErrCategoryInstance{};

std::error_code std::make_error_code(ros2_canopen::LelyBridgeErrc e)
{
  return {static_cast<int>(e), LelyBridgeErrCategoryInstance};
}

namespace ros2_canopen
{

const char * LelyBridgeErrCategory::name() const noexcept { return "LelyBridgeError"; }

std::string LelyBridgeErrCategory::message(int ev) const
{
  switch (static_cast<LelyBridgeErrc>(ev))
  {
    case LelyBridgeErrc::NotListedDevice:
      return "The CANopen device is not listed in object 1F81.";
    case LelyBridgeErrc::NoResponseOnDeviceType:
      return "No response received for upload request of object 1000.";
    case LelyBridgeErrc::DeviceTypeDifference:
      return "Value of object 1000 from CANopen device is different to value in object 1F84 "
             "(Device type).";
    case LelyBridgeErrc::VendorIdDifference:
      return "Value of object 1018:01 from CANopen device is different to value in object 1F85 "
             "(Vendor-ID).";
    case LelyBridgeErrc::HeartbeatIssue:
      return "Heartbeat event. No heartbeat message received from CANopen device.";
    case LelyBridgeErrc::NodeGuardingIssue:
      return "Node guarding event. No confirmation for guarding request received from CANopen "
             "device.";
    case LelyBridgeErrc::InconsistentProgramDownload:
      return "Objects for program download are not configured or inconsistent.";
    case LelyBridgeErrc::SoftwareUpdateRequired:
      return "Software update is required, but not allowed because of configuration or current "
             "status.";
    case LelyBridgeErrc::SoftwareDownloadFailed:
      return "Software update is required, but program download failed.";
    case LelyBridgeErrc::ConfigurationDownloadFailed:
      return "Configuration download failed.";
    case LelyBridgeErrc::StartErrorControlFailed:
      return "Heartbeat event during start error control service. No heartbeat message received "
             "from CANopen device during start error control service.";
    case LelyBridgeErrc::NmtSlaveInitiallyOperational:
      return "NMT slave was initially operational. (CANopen manager may resume operation with "
             "other CANopen devices)";
    case LelyBridgeErrc::ProductCodeDifference:
      return "Value of object 1018:02 from CANopen device is different to value in object 1F86 "
             "(Product code).";
    case LelyBridgeErrc::RevisionCodeDifference:
      return "Value of object 1018:03 from CANopen device is different to value in object 1F87 "
             "(Revision number).";
    case LelyBridgeErrc::SerialNumberDifference:
      return "Value of object 1018:04 from CANopen device is different to value in object 1F88 "
             "(Serial number).";
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

void LelyDriverBridge::OnBoot(canopen::NmtState st, char es, const ::std::string & what) noexcept
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
  lely::COSub * sub = this->dictionary_->find(idx, subidx);
  if (sub == nullptr)
  {
    std::cout << "OnRpdoWrite: id=" << (unsigned int)this->get_id() << " index=0x" << std::hex
              << (unsigned int)idx << " subindex=" << (unsigned int)subidx
              << " object does not exist" << std::endl;
    return;
  }
  uint8_t co_def = (uint8_t)sub->getType();
  uint32_t data = 0;
  if (co_def == CO_DEFTYPE_UNSIGNED8)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_UNSIGNED8>((uint8_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_UNSIGNED8>(), 1);
  }
  if (co_def == CO_DEFTYPE_INTEGER8)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_INTEGER8>((int8_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_INTEGER8>(), 1);
  }
  if (co_def == CO_DEFTYPE_UNSIGNED16)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_UNSIGNED16>((uint16_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_UNSIGNED16>(), 2);
  }
  if (co_def == CO_DEFTYPE_INTEGER16)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_INTEGER16>((int16_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_INTEGER16>(), 2);
  }
  if (co_def == CO_DEFTYPE_UNSIGNED32)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_UNSIGNED32>((uint32_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_UNSIGNED32>(), 4);
  }
  if (co_def == CO_DEFTYPE_INTEGER32)
  {
    std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
    sub->setVal<CO_DEFTYPE_INTEGER32>((int32_t)rpdo_mapped[idx][subidx]);
    std::memcpy(&data, &sub->getVal<CO_DEFTYPE_INTEGER32>(), 4);
  }
  COData codata = {idx, subidx, data};

  //  We do not care so much about missing a message, rather push them through.
  rpdo_queue->push(codata);
}

void LelyDriverBridge::OnEmcy(uint16_t eec, uint8_t er, uint8_t msef[5]) noexcept
{
  FiberDriver::OnEmcy(eec, er, msef);

  COEmcy emcy;
  emcy.eec = eec;
  emcy.er = er;
  for (int i = 0; i < 5; i++) emcy.msef[i] = msef[i];

  emcy_queue->push(emcy);
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
  lely::COSub * sub = this->dictionary_->find(data.index_, data.subindex_);
  if (sub == nullptr)
  {
    std::cout << "async_sdo_write: id=" << (unsigned int)this->get_id() << " index=0x" << std::hex
              << (unsigned int)data.index_ << " subindex=" << (unsigned int)data.subindex_
              << " object does not exist" << std::endl;
    this->sdo_write_data_promise->set_value(false);
    this->running = false;
    return sdo_write_data_promise->get_future();
  }
  uint8_t co_def = (uint8_t)sub->getType();
  try
  {
    if (co_def == CO_DEFTYPE_UNSIGNED8)
    {
      this->submit_write<uint8_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER8)
    {
      this->submit_write<int8_t>(data);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED16)
    {
      this->submit_write<uint16_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER16)
    {
      this->submit_write<int16_t>(data);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED32)
    {
      this->submit_write<uint32_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER32)
    {
      this->submit_write<int32_t>(data);
    }
  }
  catch (lely::canopen::SdoError & e)
  {
    this->sdo_read_data_promise->set_exception(lely::canopen::make_sdo_exception_ptr(
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
  lely::COSub * sub = this->dictionary_->find(data.index_, data.subindex_);
  if (sub == nullptr)
  {
    std::cout << "async_sdo_read: id=" << (unsigned int)this->get_id() << " index=0x" << std::hex
              << (unsigned int)data.index_ << " subindex=" << (unsigned int)data.subindex_
              << " object does not exist" << std::endl;
    try
    {
      throw lely::canopen::SdoError(
        this->get_id(), data.index_, data.subindex_, lely::canopen::SdoErrc::NO_OBJ);
    }
    catch (...)
    {
      this->sdo_read_data_promise->set_exception(std::current_exception());
    }
    this->running = false;
    return sdo_read_data_promise->get_future();
  }
  uint8_t co_def = (uint8_t)sub->getType();
  try
  {
    if (co_def == CO_DEFTYPE_UNSIGNED8)
    {
      this->submit_read<uint8_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER8)
    {
      this->submit_read<int8_t>(data);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED16)
    {
      this->submit_read<uint16_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER16)
    {
      this->submit_read<int16_t>(data);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED32)
    {
      this->submit_read<uint32_t>(data);
    }
    if (co_def == CO_DEFTYPE_INTEGER32)
    {
      this->submit_read<int32_t>(data);
    }
  }
  catch (lely::canopen::SdoError & e)
  {
    this->sdo_read_data_promise->set_exception(lely::canopen::make_sdo_exception_ptr(
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

std::shared_ptr<SafeQueue<COData>> LelyDriverBridge::get_rpdo_queue() { return rpdo_queue; }

std::shared_ptr<SafeQueue<COEmcy>> LelyDriverBridge::get_emcy_queue() { return emcy_queue; }

void LelyDriverBridge::tpdo_transmit(COData data)
{
  lely::COSub * sub = this->dictionary_->find(data.index_, data.subindex_);
  if (sub == nullptr)
  {
    std::cout << "async_pdo_write: id=" << (unsigned int)get_id() << " index=0x" << std::hex
              << (unsigned int)data.index_ << " subindex=" << (unsigned int)data.subindex_
              << " object does not exist" << std::endl;
    return;
  }
  uint8_t co_def = (uint8_t)sub->getType();
  try
  {
    if (co_def == CO_DEFTYPE_UNSIGNED8)
    {
      uint8_t val;
      std::memcpy(&val, &data.data_, sizeof(uint8_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_UNSIGNED8>(val);
    }
    if (co_def == CO_DEFTYPE_INTEGER8)
    {
      int8_t val;
      std::memcpy(&val, &data.data_, sizeof(int8_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_INTEGER8>(val);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED16)
    {
      uint16_t val;
      std::memcpy(&val, &data.data_, sizeof(uint16_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_UNSIGNED16>(val);
    }
    if (co_def == CO_DEFTYPE_INTEGER16)
    {
      int16_t val;
      std::memcpy(&val, &data.data_, sizeof(int16_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_INTEGER16>(val);
    }
    if (co_def == CO_DEFTYPE_UNSIGNED32)
    {
      uint32_t val;
      std::memcpy(&val, &data.data_, sizeof(uint32_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_UNSIGNED32>(val);
    }
    if (co_def == CO_DEFTYPE_INTEGER32)
    {
      int32_t val;
      std::memcpy(&val, &data.data_, sizeof(int32_t));
      tpdo_mapped[data.index_][data.subindex_] = val;
      std::scoped_lock<std::mutex> lck(this->dictionary_mutex_);
      sub->setVal<CO_DEFTYPE_INTEGER32>(val);
    }
    tpdo_mapped[data.index_][data.subindex_].WriteEvent();
    std::cout << "async_pdo_write: id=" << (unsigned int)get_id() << " index=0x" << std::hex
              << (unsigned int)data.index_ << " subindex=" << (unsigned int)data.subindex_
              << (uint32_t)data.data_ << std::endl;
  }
  catch (lely::canopen::SdoError & e)
  {
    std::cout << "async_pdo_write: id=" << (unsigned int)get_id() << " index=0x" << std::hex
              << (unsigned int)data.index_ << " subindex=" << (unsigned int)data.subindex_
              << e.what() << std::endl;
    return;
  }
}

void LelyDriverBridge::nmt_command(canopen::NmtCommand command)
{
  this->master.Command(command, nodeid);
}

uint8_t LelyDriverBridge::get_id() { return nodeid; }
}  // namespace ros2_canopen
