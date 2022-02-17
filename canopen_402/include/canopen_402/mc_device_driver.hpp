#ifndef MC_DEVICE_DRIVER_HPP
#define MC_DEVICE_DRIVER_HPP
#include <iostream>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <cstring>

#include "canopen_402/base.h"
#include "ros2_canopen_core/basic_device_driver.hpp"

using namespace ros2_canopen;
namespace canopen_402
{
    struct RemoteObject 
    {
        uint16_t index;
        uint8_t subindex;
        uint32_t data;
        CODataTypes type;
        bool pdo_mapped;
        bool valid;
    };

    class MCDeviceDriver : public BasicDeviceDriver
    {
    private:
        std::vector<std::shared_ptr<RemoteObject>> objs;

    public:
        std::shared_ptr<RemoteObject> create_remote_obj(uint16_t index, uint8_t subindex, CODataTypes type)
        {
            RemoteObject obj = {index, subindex, 0, type, false, true};
            objs.push_back(std::make_shared<RemoteObject>(obj));
        }

        void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override 
        {
            for(auto it = objs.begin(); it != objs.end(); ++it)
            {
                std::shared_ptr<RemoteObject> obj = *it;
                if(obj->index == idx && obj->subindex == subidx)
                {
                    obj->pdo_mapped == true;
                    obj->data = rpdo_mapped[idx][subidx];
                    break;
                }
            }
        }

        template<typename  T>
        void set_remote_obj(std::shared_ptr<RemoteObject> obj, T data)
        {
            T data_ = data;
            std::memcpy(&(obj->data), &data_, sizeof(T));

            COData d = {obj->index, obj->subindex, obj->data, obj->type};
            if(!obj->pdo_mapped == true)
            {
                
                auto f = this->async_sdo_write(d);
                f.wait();
            }
            else
            {
                this->tpdo_transmit(d);
            }

        }

        template<typename  T>
        T get_remote_obj(std::shared_ptr<RemoteObject> obj)
        {
            if(!obj->pdo_mapped == true)
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

        template<typename  T>
        T get_remote_obj_cached(std::shared_ptr<RemoteObject> obj)
        {
            T data;
            std::memcpy(&data, &(obj->data), sizeof(T));
            return data;
        }

        template<typename  T>
        void set_remote_obj_cached(std::shared_ptr<RemoteObject> obj, const T data)
        {
            T data_ = data;
            std::memcpy(&(obj->data), &data_, sizeof(T));
        }

        void validate_objs(){
            for(auto it = objs.begin(); it != objs.end(); ++it)
            {
                std::shared_ptr<RemoteObject> obj = *it;
                if(obj->type == CODataTypes::COData8) this->get_remote_obj<uint8_t>(obj);
                if(obj->type == CODataTypes::COData16) this->get_remote_obj<uint16_t>(obj);
                if(obj->type == CODataTypes::COData32) this->get_remote_obj<uint32_t>(obj);
            }
        }

        MCDeviceDriver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id)
            : BasicDeviceDriver(exec, master, id)
        {
            
        }

    };

}


#endif