#ifndef MC_DEVICE_DRIVER_HPP
#define MC_DEVICE_DRIVER_HPP
#include <iostream>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <cstring>

#include "canopen_402_driver/base.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"

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

    /**
     * @brief Specialised LelyDriverBridge for MotionControllers
     *
     * This class provides funtionalities necessary for interacting
     * with the canopen_402 stack from ros_canopen.
     */
    class LelyMotionControllerBridge : public LelyDriverBridge
    {
    private:
        std::vector<std::shared_ptr<RemoteObject>> objs;
        bool sync;
        double speed;
        double position;

    public:
        /**
         * @brief Create a remote obj object
         *
         * This function registers an object on the remote that is frequently used
         * to control the device. A pointer to the object is stored.
         *
         * @param [in] index            Index of remote object
         * @param [in] subindex         Subindex of remote object
         * @param [in] type             Type of remote object
         * @return std::shared_ptr<RemoteObject>
         */
        std::shared_ptr<RemoteObject> create_remote_obj(uint16_t index, uint8_t subindex, CODataTypes type)
        {
            RemoteObject obj = {index, subindex, 0, type, false, false, true};
            for (auto it = objs.begin(); it != objs.end(); ++it)
            {
                if (
                    ((*it)->index == index) &&
                    ((*it)->subindex == subindex) &&
                    ((*it)->type == type))
                {
                    return *it;
                }
            }
            std::shared_ptr<RemoteObject> objp = std::make_shared<RemoteObject>(obj);
            objs.push_back(objp);
            return objp;
        }

        /**
         * @brief Update registered objectsa on RPDO write
         *
         * This function is called when an RPDO write request is received
         * from the remote device. The funciton updates the registered objects
         * in its remote dictionary.
         *
         * @param [in] idx              Index of written object
         * @param [in] subidx           Subindex of written object
         */
        void OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
        {
            for (auto it = objs.begin(); it != objs.end(); ++it)
            {
                std::shared_ptr<RemoteObject> obj = *it;
                if (obj->index == idx && obj->subindex == subidx)
                {
                    if (obj->type == CODataTypes::COData8)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint8_t>();
                    else if (obj->type == CODataTypes::COData16)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint16_t>();
                    else if (obj->type == CODataTypes::COData32)
                        obj->data = rpdo_mapped[idx][subidx].Read<uint32_t>();
                    break;
                }
            }

            if (idx == 0x606C && subidx == 0x0)
            {
                speed = rpdo_mapped[idx][subidx].Read<int32_t>();
            }

            if (idx == 0x6064 && subidx == 0x0)
            {
                position = rpdo_mapped[idx][subidx].Read<int32_t>();
            }
        }

        /**
         * @brief Set the remote obj
         *
         * Set the data of the remote object. This function will write the
         * data passed to it to the local cache and the remote dictionary.
         *
         * @tparam T                Datatype of the object
         * @param [in] obj          Shared pointer to the object
         * @param [in] data         Data to be written
         */
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
                this->tpdo_mapped[obj->index][obj->subindex].WriteEvent();
            }
        }
        /**
         * @brief Get the remote obj
         *
         * Gets the data stored in the remote object. If the object is
         * PDO mapped, the cached data is returned, else the data is fetched
         * via SDO request.
         *
         * @tparam T                Datatype of the object
         * @param [in] obj          Pointer to object
         * @return T                Data that was read
         */
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
                    RCLCPP_ERROR(rclcpp::get_logger(name_), e.what());
                    obj->valid = false;
                }
            }
            T data;
            std::memcpy(&data, &(obj->data), sizeof(T));
            return data;
        }
        /**
         * @brief Get an object from local cache
         * 
         * @tparam T 
         * @param obj 
         * @return T 
         */
        template <typename T>
        T get_remote_obj_cached(std::shared_ptr<RemoteObject> obj)
        {
            T data;
            std::memcpy(&data, &(obj->data), sizeof(T));
            return data;
        }

        /**
         * @brief Set an object in local cache
         * 
         * @tparam T 
         * @param obj 
         * @param data 
         */
        template <typename T>
        void set_remote_obj_cached(std::shared_ptr<RemoteObject> obj, const T data)
        {
            T data_ = data;
            std::memcpy(&(obj->data), &data_, sizeof(T));
        }

        /**
         * @brief Validate objects in dictionary
         *
         * Validates whether objects exist and whether they are pdo mapped.
         *
         */
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
                        this->tpdo_mapped[obj->index][obj->subindex].Read<uint8_t>();
                        break;
                    case CODataTypes::COData16:
                        this->tpdo_mapped[obj->index][obj->subindex].Read<uint16_t>();
                        break;
                    case CODataTypes::COData32:
                        this->tpdo_mapped[obj->index][obj->subindex].Read<uint32_t>();
                        break;
                    default:
                        throw lely::canopen::SdoError(
                            this->get_id(),
                            obj->index,
                            obj->subindex,
                            std::make_error_code(std::errc::function_not_supported),
                            "Unkown used, type must be 8, 16 or 32.");
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
                    default:
                        throw lely::canopen::SdoError(
                            this->get_id(),
                            obj->index,
                            obj->subindex,
                            std::make_error_code(std::errc::function_not_supported),
                            "Unkown used, type must be 8, 16 or 32.");
                        break;
                    }
                    obj->rpdo_mapped = true;
                }
                catch (lely::canopen::SdoError &e)
                {
                    obj->rpdo_mapped = false;
                }

                try
                {
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
                    default:
                        break;
                    }
                }
                catch (lely::canopen::SdoError &e)
                {
                    RCLCPP_ERROR(rclcpp::get_logger(name_), "Could not fetch data. %s", e.what());
                }
                RCLCPP_INFO(rclcpp::get_logger(name_), "Initialised object: node_id %hu, index %x, subindex %hhu, data %u, RPDO: %s, TPDO: %s",
                    this->get_id(),
                    obj->index,
                    obj->subindex,
                    obj->data,
                    (obj->rpdo_mapped ? "yes" : "no"),
                    (obj->tpdo_mapped ? "yes" : "no")
                    );
            }
        }

        /**
         * @brief Get the speed object
         *
         * @return double
         */
        double get_speed()
        {
            return speed;
        }

        /**
         * @brief Get the position object
         *
         * @return double
         */
        double get_position()
        {
            return position;
        }

        /**
         * @brief Construct a new LelyMotionControllerBridge object
         * 
         * @param [in] exec 
         * @param [in] master 
         * @param [in] id 
         */
        LelyMotionControllerBridge(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id, std::string name);
    };
}
#endif
