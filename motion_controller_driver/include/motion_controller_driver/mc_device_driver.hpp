#ifndef MC_DEVICE_DRIVER_HPP
#define MC_DEVICE_DRIVER_HPP
#include <iostream>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <cstring>

#include "motion_controller_driver/base.hpp"
#include "ros2_canopen_core/basic_device_driver.hpp"

using namespace ros2_canopen;
namespace ros2_canopen
{
    struct RemoteObject
    {
        uint16_t index;
        uint8_t subindex;
        uint32_t data;
        CODataTypes type;
        bool tpdo_mapped;
        bool rpdo_mapped;
        bool valid;
    };

    class MCDeviceDriver : public BasicDeviceDriver
    {
    private:
        std::vector<std::shared_ptr<RemoteObject>> objs;
        bool sync;

    public:
        std::shared_ptr<RemoteObject> create_remote_obj(uint16_t index, uint8_t subindex, CODataTypes type)
        {
            RemoteObject obj = {index, subindex, 0, type, false, false, true};
            std::shared_ptr<RemoteObject> objp = std::make_shared<RemoteObject>(obj);
            objs.push_back(objp);
            return objp;
        }

        void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
        {
            for (auto it = objs.begin(); it != objs.end(); ++it)
            {
                std::shared_ptr<RemoteObject> obj = *it;
                if (obj->index == idx && obj->subindex == subidx)
                {
                    if(obj->type == CODataTypes::COData8)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint8_t>();
                    else if(obj->type == CODataTypes::COData16)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint16_t>();
                    else if(obj->type == CODataTypes::COData32)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint32_t>();
                    break;
                }
            }
        }

        template <typename T>
        void set_remote_obj(std::shared_ptr<RemoteObject> obj, T data)
        {
            T data_ = data;
            std::memcpy(&(obj->data), &data_, sizeof(T));

            COData d = {obj->index, obj->subindex, obj->data, obj->type};
            if (!obj->tpdo_mapped)
            {

                auto f = this->async_sdo_write(d);
                f.wait();
            }
            else
            {
                this->tpdo_mapped[obj->index][obj->subindex] = data;
                if(!sync) 
                    this->tpdo_mapped[obj->index][obj->subindex].WriteEvent();
            }
        }

        template <typename T>
        T get_remote_obj(std::shared_ptr<RemoteObject> obj)
        {
            if (!obj->rpdo_mapped)
            {
                COData d = {obj->index, obj->subindex, 0U, obj->type};
                auto f = this->async_sdo_read(d);
                f.wait();
                try
                {
                    obj->data = f.get().data_;
                }
                catch (std::exception &e)
                {
                    obj->valid = false;
                }
            }
            T data;
            std::memcpy(&data, &(obj->data), sizeof(T));
            return data;
        }

        template <typename T>
        T get_remote_obj_cached(std::shared_ptr<RemoteObject> obj)
        {
            T data;
            std::memcpy(&data, &(obj->data), sizeof(T));
            return data;
        }

        template <typename T>
        void set_remote_obj_cached(std::shared_ptr<RemoteObject> obj, const T data)
        {
            T data_ = data;
            std::memcpy(&(obj->data), &data_, sizeof(T));
        }

        void validate_objs()
        {
            for (auto it = objs.begin(); it != objs.end(); ++it)
            {
                std::shared_ptr<RemoteObject> obj = *it;

                try
                {
                    switch (obj->type)
                    {
                    case CODataTypes::COData8:
                        obj->rpdo_mapped = this->tpdo_mapped[obj->index][obj->subindex].Read<uint8_t>();
                        break;
                    case CODataTypes::COData16:
                        obj->rpdo_mapped = this->tpdo_mapped[obj->index][obj->subindex].Read<uint16_t>();
                        break;
                    case CODataTypes::COData32:
                        obj->rpdo_mapped = this->tpdo_mapped[obj->index][obj->subindex].Read<uint32_t>();
                        break;
                    }
                    obj->tpdo_mapped = true;
                }
                catch (lely::canopen::SdoError &e)
                {
                    obj->tpdo_mapped = false;
                }

                try
                {
                    switch (obj->type)
                    {
                    case CODataTypes::COData8:
                        obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint8_t>();
                        break;
                    case CODataTypes::COData16:
                        obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint16_t>();
                        break;
                    case CODataTypes::COData32:
                        obj->rpdo_mapped = this->rpdo_mapped[obj->index][obj->subindex].Read<uint32_t>();
                        break;
                    }
                    obj->rpdo_mapped = true;
                }
                catch (lely::canopen::SdoError &e)
                {
                    obj->rpdo_mapped = false;
                }

                switch (obj->type)
                {
                case CODataTypes::COData8:
                    obj->data = get_remote_obj<uint8_t>(obj);
                    break;
                case CODataTypes::COData16:
                    obj->data = get_remote_obj<uint16_t>(obj);
                    break;
                case CODataTypes::COData32:
                    obj->data = get_remote_obj<uint32_t>(obj);
                    break;
                }
                std::cout << "Initialised object :" 
                    << this->get_id() << " " 
                    << std::hex << obj->index << " " 
                    << std::dec << obj->subindex << " " 
                    << obj->data << " "
                    << "RPDO: " << (obj->rpdo_mapped ? "yes" : "no") << " "
                    << "TPDO: " << (obj->tpdo_mapped ? "yes" : "no") << " "
                    << std::endl;
            }
        }

        MCDeviceDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id)
            : BasicDeviceDriver(exec, master, id)
        {
            sync = true;
        }
    };

}
#endif
