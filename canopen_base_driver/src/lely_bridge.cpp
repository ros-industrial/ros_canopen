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
#include "canopen_base_driver/lely_bridge.hpp"

namespace ros2_canopen
{

void LelyBridge::OnState(canopen::NmtState state) noexcept
{
  canopen::NmtState st = state;
  // We assume 1F80 bit 2 is false. All slaves are put into Operational after boot-up.
  // Lelycore does not track NMT states in this mode except BOOTUP.
  if (st == canopen::NmtState::BOOTUP) {
    st = canopen::NmtState::START;
  }

  if (!nmt_state_is_set.load()) {
    // We do not care so much about missing a message, rather push them through.
    std::unique_lock<std::mutex> lk(nmt_mtex, std::defer_lock);
    if (lk.try_lock()) {
      nmt_state_is_set.store(true);
      nmt_state_promise.set_value(st);
    }
  }
}

void LelyBridge::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
{
  uint32_t data = (uint32_t)rpdo_mapped[idx][subidx];
  COData codata = {idx, subidx, data, CODataTypes::CODataUnkown};

  // We do not care so much about missing a message, rather push them through.
  std::unique_lock<std::mutex> lk(pdo_mtex, std::defer_lock);
  if (lk.try_lock()) {
    if (!rpdo_is_set.load()) {
      rpdo_is_set.store(true);
      rpdo_promise.set_value(codata);
    }
  }
}

std::future<bool> LelyBridge::async_sdo_write(COData data)
{
  std::unique_lock<std::mutex> lck(sdo_mutex);
  if (running) {
    sdo_cond.wait(lck);
  }
  running = true;

  sdo_write_data_promise = std::make_shared<std::promise<bool>>();
  if (data.type_ == CODataTypes::COData8) {
    this->SubmitWrite(
      data.index_, data.subindex_, (uint8_t)data.data_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec) mutable
      {
        if (ec) {
          this->sdo_write_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id,
              idx,
              subidx,
              ec,
              "AsyncDownload")
          );
        } else {
          this->sdo_write_data_promise->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  } else if (data.type_ == CODataTypes::COData16) {
    this->SubmitWrite(
      data.index_, data.subindex_, (uint16_t)data.data_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec) mutable
      {
        if (ec) {
          this->sdo_write_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id, idx, subidx, ec, "AsyncDownload")
          );
        } else {
          this->sdo_write_data_promise->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  } else if (data.type_ == CODataTypes::COData32) {
    this->SubmitWrite(
      data.index_, data.subindex_, (uint32_t)data.data_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec) mutable
      {
        if (ec) {
          this->sdo_write_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id, idx, subidx, ec, "AsyncDownload")
          );
        } else {
          this->sdo_write_data_promise->set_value(true);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }

  return sdo_write_data_promise->get_future();
}

std::future<COData> LelyBridge::async_sdo_read(COData data)
{
  std::unique_lock<std::mutex> lck(sdo_mutex);
  if (running) {
    sdo_cond.wait(lck);
  }
  running = true;

  sdo_read_data_promise = std::make_shared<std::promise<COData>>();
  if (data.type_ == CODataTypes::COData8) {
    this->SubmitRead<uint8_t>(
      data.index_, data.subindex_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec, uint8_t value) mutable
      {
        if (ec) {
          this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id, idx, subidx, ec, "AsyncUpload")
          );
        } else {
          COData d = {idx, subidx, value, CODataTypes::COData16};
          this->sdo_read_data_promise->set_value(d);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  } else if (data.type_ == CODataTypes::COData16) {
    this->SubmitRead<uint16_t>(
      data.index_, data.subindex_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec, uint16_t value) mutable
      {
        if (ec) {
          this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id, idx, subidx, ec, "AsyncUpload")
          );
        } else {
          COData d = {idx, subidx, value, CODataTypes::COData16};
          this->sdo_read_data_promise->set_value(d);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  } else if (data.type_ == CODataTypes::COData32) {
    this->SubmitRead<uint16_t>(
      data.index_, data.subindex_,
      [this](uint8_t id, uint16_t idx, uint8_t subidx,
      ::std::error_code ec, uint32_t value) mutable
      {
        if (ec) {
          this->sdo_read_data_promise->set_exception(
            lely::canopen::make_sdo_exception_ptr(
              id, idx, subidx, ec, "AsyncUpload")
          );
        } else {
          COData d = {idx, subidx, value, CODataTypes::COData16};
          this->sdo_read_data_promise->set_value(d);
        }
        std::unique_lock<std::mutex> lck(this->sdo_mutex);
        this->running = false;
        this->sdo_cond.notify_one();
      },
      20ms);
  }
  return sdo_read_data_promise->get_future();
}

std::future<canopen::NmtState> LelyBridge::async_request_nmt()
{
  std::scoped_lock<std::mutex> lk(nmt_mtex);
  nmt_state_is_set.store(false);
  nmt_state_promise = std::promise<canopen::NmtState>();
  return nmt_state_promise.get_future();
}

std::future<COData> LelyBridge::async_request_rpdo()
{
  std::scoped_lock<std::mutex> lk(pdo_mtex);
  rpdo_is_set.store(false);
  rpdo_promise = std::promise<COData>();
  return rpdo_promise.get_future();
}

void LelyBridge::tpdo_transmit(COData data)
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

void LelyBridge::nmt_command(canopen::NmtCommand command)
{
  this->master.Command(command, nodeid);
}

uint8_t LelyBridge::get_id()
{
  return nodeid;
}
}  // namespace ros2_canopen
