
#include "ros2_canopen_core/device.hpp"


namespace ros2_canopen {
    class ProxyCANopenDriver : public canopen::FiberDriver {
    public:
        ProxyCANopenDriver(ev::Executor& exec,
            canopen::AsyncMaster& master,
            uint32_t node_id) : canopen::FiberDriver(exec, master, node_id) {}
    private:
        // This function gets called when the boot-up process of the slave completes.
        // The 'st' parameter contains the last known NMT state of the slave
        // (typically pre-operational), 'es' the error code (0 on success), and 'what'
        // a description of the error, if any.
        void
        OnBoot(canopen::NmtState /*st*/, char es,
                const std::string& what) noexcept override {
            if (!es || es == 'L') {
            std::cout << "slave " << static_cast<int>(id()) << " booted sucessfully"
                        << std::endl;
            } else {
            std::cout << "slave " << static_cast<int>(id())
                        << " failed to boot: " << what << std::endl;
            }
        }

        // This function gets called during the boot-up process for the slave. The
        // 'res' parameter is the function that MUST be invoked when the configuration
        // is complete. Because this function runs as a task inside a coroutine, it
        // can suspend itself and wait for an asynchronous function, such as an SDO
        // request, to complete.
        void
        OnConfig(std::function<void(std::error_code ec)> res) noexcept override {
            try {
            // Perform a few SDO write requests to configure the slave. The
            // AsyncWrite() function returns a future which becomes ready once the
            // request completes, and the Wait() function suspends the coroutine for
            // this task until the future is ready.

            // Configure the slave to monitor the heartbeat of the master (node-ID 1)
            // with a timeout of 2000 ms.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, (1 << 16) | 2000));
            // Configure the slave to produce a heartbeat every 1000 ms.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 1000));
            // Configure the heartbeat consumer on the master.
            ConfigHeartbeat(std::chrono::milliseconds(2000));

            // Reset object 4000:00 and 4001:00 on the slave to 0.
            Wait(AsyncWrite<uint32_t>(0x4000, 0, 0));
            Wait(AsyncWrite<uint32_t>(0x4001, 0, 0));

            // Report success (empty error code).
            res({});
            } catch (canopen::SdoError& e) {
            // If one of the SDO requests resulted in an error, abort the
            // configuration and report the error code.
            res(e.code());
            }
        }

        // This function is similar to OnConfg(), but it gets called by the
        // AsyncDeconfig() method of the master.
        void
        OnDeconfig(std::function<void(std::error_code ec)> res) noexcept override {
            try {
            // Disable the heartbeat consumer on the master.
            ConfigHeartbeat(std::chrono::milliseconds(0));
            // Disable the heartbeat producer on the slave.
            Wait(AsyncWrite<uint16_t>(0x1017, 0, 0));
            // Disable the heartbeat consumer on the slave.
            Wait(AsyncWrite<uint32_t>(0x1016, 1, 0));
            res({});
            } catch (canopen::SdoError& e) {
            res(e.code());
            }
        }

        // This function gets called every time a value is written to the local object
        // dictionary of the master by an RPDO (or SDO, but that is unlikely for a
        // master), *and* the object has a known mapping to an object on the slave for
        // which this class is the driver. The 'idx' and 'subidx' parameters are the
        // object index and sub-index of the object on the slave, not the local object
        // dictionary of the master.
        void
        OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override {
            if (idx == 0x4001 && subidx == 0) {
            // Obtain the value sent by PDO from object 4001:00 on the slave.
            uint32_t val = rpdo_mapped[0x4001][0];
            // Increment the value and store it to an object in the local object
            // dictionary that will be sent by TPDO to object 4000:00 on the slave.
            tpdo_mapped[0x4000][0] = ++val;
            std::cout << "master val: " << std::endl;
            }
        }
    };

    class ProxyDriverWrapper : public CANopenDriverWrapper {
    public:
        void init(ev::Executor& exec,
            canopen::AsyncMaster& master,
            uint32_t node_id) noexcept override {

            driver_ = 
                std::make_unique<ProxyCANopenDriver>(exec, master, node_id);
        }
    private:
        std::shared_ptr<ProxyCANopenDriver> driver_;
    };
}   // end namespace ros2_canopen

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros2_canopen::ProxyDriverWrapper, ros2_canopen::CANopenDriverWrapper)