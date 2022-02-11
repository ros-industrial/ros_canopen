/*
 *  Copyright 2022 Christoph Hellmann Santos
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */
#include "ros2_canopen_core/basic_device_driver.hpp"

namespace ros2_canopen
{

    void BasicDeviceDriver::sdo_write_event()
    {
        // Create and wait
        sdo_write_promise = ev::Promise<COData, std::exception_ptr>();
        sdo_write_data_promise = std::promise<bool>();
        sdo_write_future = sdo_write_promise.get_future();
        sdo_write_lock.unlock();
        sdo_write_future.submit(this->GetStrand(), std::bind(&BasicDeviceDriver::sdo_write_event_cb, this));
    }

    void BasicDeviceDriver::sdo_write_event_cb()
    {
        sdo_write_lock.lock();
        auto d = sdo_write_future.get().value();

        // Prepare read and submit and wait
        bool res;
        if (d.type_ == CODataTypes::COData8)
        {
            auto f2 = AsyncWrite<uint8_t>(d.index_, d.subindex_, (uint8_t)d.data_);
            Wait(f2);

            auto r = !f2.get().has_error();
            res = r;
        }
        else if (d.type_ == CODataTypes::COData16)
        {
            auto f2 = AsyncWrite<uint16_t>(d.index_, d.subindex_, (uint16_t)d.data_);
            Wait(f2);

            auto r = !f2.get().has_error();
            res = r;
        }
        else if (d.type_ == CODataTypes::COData32)
        {
            auto f2 = AsyncWrite<uint32_t>(d.index_, d.subindex_, (uint32_t)d.data_);
            Wait(f2);

            auto r = !f2.get().has_error();
            res = r;
        }

        // Handle Result and set response
        sdo_write_data_promise.set_value(res);

        // Reschedule task for next sdo request.
        this->Defer(std::bind(&BasicDeviceDriver::sdo_write_event, this));
    }

    void BasicDeviceDriver::sdo_read_event()
    {
        // Create and wait
        sdo_read_promise = ev::Promise<COData, std::exception_ptr>();
        sdo_read_data_promise = std::promise<COData>();
        sdo_read_future = sdo_read_promise.get_future();
        sdo_read_lock.unlock();
        sdo_read_future.submit(this->GetStrand(), std::bind(&BasicDeviceDriver::sdo_read_event_cb, this));
    }

    void BasicDeviceDriver::sdo_read_event_cb()
    {
        sdo_read_lock.lock();
        COData res;
        // Prepare read and submit and wait
        auto d = sdo_read_future.get().value();
        if (d.type_ == CODataTypes::COData8)
        {
            auto f2 = AsyncRead<uint8_t>(d.index_, d.subindex_);

            Wait(f2);

            auto r = f2.get().value();
            res = {d.index_, d.subindex_, r, CODataTypes::COData8};
        }
        else if (d.type_ == CODataTypes::COData16)
        {
            auto f2 = AsyncRead<uint16_t>(d.index_, d.subindex_);

            Wait(f2);

            auto r = f2.get().value();
            res = {d.index_, d.subindex_, r, CODataTypes::COData16};
        }
        else if (d.type_ == CODataTypes::COData32)
        {
            auto f2 = AsyncRead<uint32_t>(d.index_, d.subindex_);

            Wait(f2);
        }

        // Handle Result and set response
        sdo_read_data_promise.set_value(res);

        // Reschedule task for next sdo request.
        this->Defer(std::bind(&BasicDeviceDriver::sdo_read_event, this));
    }

    void BasicDeviceDriver::OnState(canopen::NmtState state) noexcept
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

    void BasicDeviceDriver::OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept
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

    std::future<bool> BasicDeviceDriver::async_sdo_write(COData data)
    {
        std::scoped_lock<std::mutex> lk2(sdo_write_mutex);
        sdo_write_promise.set(data);
        return sdo_write_data_promise.get_future();
    }

    std::future<COData> BasicDeviceDriver::async_sdo_read(COData data)
    {
        std::scoped_lock<std::mutex> lk2(sdo_read_mutex);
        sdo_read_promise.set(data);
        return sdo_read_data_promise.get_future();
    }

    std::future<canopen::NmtState> BasicDeviceDriver::async_request_nmt()
    {
        std::scoped_lock<std::mutex> lk(nmt_mtex);
        nmt_state_is_set.store(false);
        nmt_state_promise = std::promise<canopen::NmtState>();
        return nmt_state_promise.get_future();
    }

    std::future<COData> BasicDeviceDriver::async_request_rpdo()
    {
        std::scoped_lock<std::mutex> lk(pdo_mtex);
        rpdo_is_set.store(false);
        rpdo_promise = std::promise<COData>();
        return rpdo_promise.get_future();
    }

    void BasicDeviceDriver::tpdo_transmit(COData data)
    {
        TPDOWriteTask task(this->GetStrand());
        task.driver = this;
        task.data = data;
        {
            this->master.GetExecutor().post(
                [&task, this]{
                    this->GetStrand().post(task);
                }
            );
        }
        // Wait for task to finish.
        std::scoped_lock<std::mutex> lk(task.mtx);
    }

    void BasicDeviceDriver::nmt_command(canopen::NmtCommand command)
    {
        this->master.Command(command, nodeid);
    }

    uint8_t BasicDeviceDriver::get_id()
    {
        return nodeid;
    }
}