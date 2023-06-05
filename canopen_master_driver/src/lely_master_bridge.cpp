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
#include "canopen_master_driver/lely_master_bridge.hpp"
#include <chrono>
#include <lely/co/dcf.hpp>

using namespace std::literals::chrono_literals;

namespace ros2_canopen /* constant-expression */
{
std::future<bool> LelyMasterBridge::async_write_sdo(uint8_t id, COData data, uint8_t datatype)
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
    switch (datatype)
    {
      case CO_DEFTYPE_INTEGER8:
        submit_write_sdo<int8_t>(id, data.index_, data.subindex_, data.data_);
        break;
      case CO_DEFTYPE_INTEGER16:
        submit_write_sdo<int16_t>(id, data.index_, data.subindex_, data.data_);
        break;
      case CO_DEFTYPE_INTEGER32:
        submit_write_sdo<int32_t>(id, data.index_, data.subindex_, data.data_);
        break;
      case CO_DEFTYPE_UNSIGNED8:
        submit_write_sdo<uint8_t>(id, data.index_, data.subindex_, data.data_);
        break;
      case CO_DEFTYPE_UNSIGNED16:
        submit_write_sdo<uint16_t>(id, data.index_, data.subindex_, data.data_);
        break;
      case CO_DEFTYPE_UNSIGNED32:
        submit_write_sdo<uint32_t>(id, data.index_, data.subindex_, data.data_);
        break;
      default:
        throw lely::canopen::SdoError(
          id, data.index_, data.subindex_, lely::canopen::SdoErrc::ERROR, "Unknown datatype");
        break;
    }
  }
  catch (...)
  {
    this->sdo_write_data_promise->set_exception(std::current_exception());
    this->running = false;
  }

  return sdo_write_data_promise->get_future();
}

std::future<COData> LelyMasterBridge::async_read_sdo(uint8_t id, COData data, uint8_t datatype)
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
    switch (datatype)
    {
      case CO_DEFTYPE_INTEGER8:
        submit_read_sdo<int8_t>(id, data.index_, data.subindex_);
        break;
      case CO_DEFTYPE_INTEGER16:
        submit_read_sdo<int16_t>(id, data.index_, data.subindex_);
        break;
      case CO_DEFTYPE_INTEGER32:
        submit_read_sdo<int32_t>(id, data.index_, data.subindex_);
        break;
      case CO_DEFTYPE_UNSIGNED8:
        submit_read_sdo<uint8_t>(id, data.index_, data.subindex_);
        break;
      case CO_DEFTYPE_UNSIGNED16:
        submit_read_sdo<uint16_t>(id, data.index_, data.subindex_);
        break;
      case CO_DEFTYPE_UNSIGNED32:
        submit_read_sdo<uint32_t>(id, data.index_, data.subindex_);
        break;
      default:
        throw lely::canopen::SdoError(
          id, data.index_, data.subindex_, lely::canopen::SdoErrc::ERROR, "Unknown datatype");
        break;
    }
  }
  catch (...)
  {
    this->sdo_read_data_promise->set_exception(std::current_exception());
    this->running = false;
  }
  return sdo_read_data_promise->get_future();
}

std::future<bool> LelyMasterBridge::async_write_nmt(uint8_t id, uint8_t command)
{
  lely::canopen::NmtCommand command_ = static_cast<lely::canopen::NmtCommand>(command);
  switch (command_)
  {
    case lely::canopen::NmtCommand::ENTER_PREOP:
    case lely::canopen::NmtCommand::RESET_COMM:
    case lely::canopen::NmtCommand::RESET_NODE:
    case lely::canopen::NmtCommand::START:
    case lely::canopen::NmtCommand::STOP:
      this->Command(command_, id);
      break;
    default:
      break;
  }

  nmt_promise.set_value(true);
  return nmt_promise.get_future();
}
}  // namespace ros2_canopen
