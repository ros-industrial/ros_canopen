#include <lely/ev/loop.hpp>
#if _WIN32
#include <lely/io2/win32/ixxat.hpp>
#include <lely/io2/win32/poll.hpp>
#elif defined(__linux__)
#include <lely/io2/posix/poll.hpp>
#else
#error This file requires Windows or Linux.
#endif
#include <yaml-cpp/yaml.h>
#include "ros2_canopen_core/device_manager.hpp"


bool DeviceManager::load_component(const std::string& pkg_name, const std::string& plugin_name, uint32_t node_id, std::string& node_name) {
    ComponentResource component;
    std::vector<ComponentResource> components = this->get_component_resources(pkg_name);
    for(auto it = components.begin(); it != components.end(); ++it)
    {
        if(it->first.compare(plugin_name) == 0)
        {
            auto factory_node = this->create_component_factory(*it);
            rclcpp::NodeOptions opts;
            opts.use_global_arguments(false);
            std::vector<std::string> remap_rules;
            remap_rules.push_back("--ros-args");
            remap_rules.push_back("-r");
            remap_rules.push_back("__node:=" + node_name);
            opts.arguments(remap_rules);
            rclcpp_components::NodeInstanceWrapper wrapper = factory_node->create_node_instance(opts);
            auto node_instance = std::static_pointer_cast<ros2_canopen::CANopenDriverWrapper>(wrapper.get_node_instance());
            
            drivers_.insert({node_id, node_instance});
            
            exec_->post([this, node_id, node_instance](){
                node_instance->init(*exec_, *can_master_, node_id);
            });
            

            RCLCPP_INFO(this->get_logger(), "Initialising loaded %s", plugin_name.c_str());
            if(auto exec = executor_.lock())
            {
                exec->add_node(wrapper.get_node_base_interface(), true);
                RCLCPP_INFO(this->get_logger(), "Added %s of type %s to executor", node_name.c_str(), plugin_name.c_str());
            }
                
            return true;
        }
    }
    return false;
}

bool DeviceManager::load_driver(std::string& package_name, std::string& device_name,
        uint32_t node_id, std::string& node_name) {

    std::string plugin_name = device_name;
    return this->load_component(package_name, plugin_name, node_id, node_name);
}

bool DeviceManager::init_devices_from_config(io::Timer& timer,
    io::CanChannel& chan,
    ev::Executor& exec,
    std::string& dcf_txt,
    std::string& dcf_config) {

    YAML::Node node;
    try {
        node = YAML::LoadFile(dcf_config.c_str());
    }
    catch(const YAML::BadFile& ex) {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return false;
    }
    
	for (
		YAML::const_iterator it = node.begin();
		it != node.end();
		it++) {
		// Get toplevel node name
		std::string driver_name = it->first.as<std::string>();
        // Device config
        YAML::Node config = it->second;
        int node_id = config["node_id"].as<int>();

        // init master
        if (driver_name.find("master") != std::string::npos) {
            // TODO: load master component
            try {
                can_master_ = 
                    std::make_shared<ros2_canopen::MasterDevice>(timer, chan,
                        dcf_txt, node_id, driver_name);
            } catch(const std::exception& e) {
                std::cerr << e.what() << '\n';
            }            
        } else {
			// load the driver
			std::string plugin_name = config["driver"].as<std::string>();
            std::string package_name = config["package"].as<std::string>();
            //TODO: if one of the driver fails to load,
            // should the state change or exit with FAILURE?
            std::thread t([this, &package_name, &plugin_name, node_id, &driver_name]() {
                load_driver(package_name, plugin_name, node_id, driver_name);
            });
            t.join();            
		}
	}
    return true;
}

bool DeviceManager::init() {
    std::string can_interface_name;
    std::string dcf_txt;
    std::string dcf_config;

    this->get_parameter("can_interface_name", can_interface_name);
	this->get_parameter("master_config", dcf_txt);
	this->get_parameter("bus_config", dcf_config);

    io::IoGuard io_guard;
#if _WIN32
    // Load vcinpl2.dll (or vcinpl.dll if CAN FD is disabled).
    io::IxxatGuard ixxat_guard;
#endif
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    
    exec_ = std::make_shared<ev::Executor>(loop.get_executor());
    io::Timer timer(poll, *exec_, CLOCK_MONOTONIC);
#if _WIN32
    // Create an IXXAT CAN controller and channel. The VCI requires us to
    // explicitly specify the bitrate and restart the controller.
    io::IxxatController ctrl(0, 0, io::CanBusFlag::NONE, 125000);
    ctrl.restart();
    io::IxxatChannel chan(ctx, exec);
#elif defined(__linux__)
    io::CanController ctrl(can_interface_name.c_str());
    io::CanChannel chan(poll, *exec_);
#endif
    chan.open(ctrl);

    bool res = init_devices_from_config(timer, chan, *exec_,
        dcf_txt, dcf_config);
    if(!res) {
        // what's the difference between FAILURE AND ERROR?
        return false;
    }

    // Create a signal handler.
    io::SignalSet sigset(poll, *exec_);
    // Watch for Ctrl+C or process termination.
    sigset.insert(SIGHUP);
    sigset.insert(SIGINT);
    sigset.insert(SIGTERM);

    // Submit a task to be executed when a signal is raised. We don't care which.
    sigset.submit_wait([&](int /*signo*/) {
        // If the signal is raised again, terminate immediately.
        sigset.clear();
        // Perform a clean shutdown.
        ctx.shutdown();
    });

    // Start the NMT service of the master by pretending to receive a 'reset
    // node' command.
    can_master_->Reset();

    loop.run();
    return true;
}


int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_manager = std::make_shared<DeviceManager>(exec);
    std::thread spinThread([&device_manager]() {
        std::cout << "Init success: " << device_manager->init() << std::endl;
    });
    exec->add_node(device_manager);
    exec->spin();
    spinThread.join();
    return 0;
}
