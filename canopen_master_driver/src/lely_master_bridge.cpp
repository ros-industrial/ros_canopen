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

using namespace std::literals::chrono_literals;

namespace ros2_canopen
{
    std::future<bool> LelyMasterBridge::async_write_sdo(uint8_t id, COData data)
    {
        std::unique_lock<std::mutex> lck(sdo_mutex);
        if (running)
        {
            sdo_cond.wait(lck);
        }
        running = true;

        sdo_write_data_promise = std::make_shared<std::promise<bool>>();
        if (data.type_ == CODataTypes::COData8)
        {
            this->SubmitWrite(
                this->GetExecutor(),
                id,
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
                this->GetExecutor(),
                id,
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
                this->GetExecutor(),
                id,
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

        return sdo_write_data_promise->get_future();
    }

    std::future<COData> LelyMasterBridge::async_read_sdo(uint8_t id, COData data)
    {
        std::unique_lock<std::mutex> lck(sdo_mutex);
        if (running)
        {
            sdo_cond.wait(lck);
        }
        running = true;

        sdo_read_data_promise = std::make_shared<std::promise<COData>>();
        if (data.type_ == CODataTypes::COData8)
        {
            this->SubmitRead<uint8_t>(
                this->GetExecutor(),
                id,
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
                this->GetExecutor(),
                id,
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
                this->GetExecutor(),
                id,
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
}