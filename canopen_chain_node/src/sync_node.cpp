#include <canopen_master/bcm_sync.h>
#include <socketcan_interface/string.h>
#include <canopen_master/can_layer.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

template<typename T > std::string join(const T &container, const std::string &delim){
    if(container.empty()) return std::string();
    std::stringstream sstr;
    typename T::const_iterator it = container.begin();
    sstr << *it;
    for(++it; it != container.end(); ++it){
        sstr << delim << *it;
    }
    return sstr.str();
}
void report_diagnostics(canopen::LayerStack &sync, diagnostic_updater::DiagnosticStatusWrapper &stat){
    canopen::LayerReport r;
    sync.read(r);
    sync.diag(r);
    if(sync.getLayerState() !=canopen::Layer::Off  &&  r.bounded<canopen::LayerStatus::Unbounded>()){ // valid
        stat.summary(r.get(), r.reason());
        for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
            stat.add(it->first, it->second);
        }
        if(!r.bounded<canopen::LayerStatus::Warn>()){
            canopen::LayerStatus s;
            sync.recover(s);
        }
    }else{
        stat.summary(stat.WARN, "sync not initilized");
        canopen::LayerStatus s;
        sync.init(s);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "canopen_sync_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ros::NodeHandle sync_nh(nh_priv, "sync");
    int sync_ms;
    if(!sync_nh.getParam("interval_ms", sync_ms) || sync_ms <=0){
        ROS_ERROR_STREAM("Sync interval  "<< sync_ms << " is invalid");
        return 1;
    }

    int sync_overflow = 0;
    if(!sync_nh.getParam("overflow", sync_overflow)){
        ROS_WARN("Sync overflow was not specified, so overflow is disabled per default");
    }
    if(sync_overflow == 1 || sync_overflow > 240){
        ROS_ERROR_STREAM("Sync overflow  "<< sync_overflow << " is invalid");
        return 1;
    }


    std::string can_device;
    if(!nh_priv.getParam("bus/device",can_device)){
        ROS_ERROR("Device not set");
        return 1;
    }

    can::SocketCANDriverSharedPtr driver = boost::make_shared<can::SocketCANDriver>();
    canopen::SyncProperties sync_properties(can::MsgHeader(sync_nh.param("sync_id", 0x080)), sync_ms, sync_overflow);

    boost::shared_ptr<canopen::BCMsync> sync = boost::make_shared<canopen::BCMsync>(can_device, driver, sync_properties);

    std::vector<int> nodes;

    if(sync_nh.getParam("monitored_nodes",nodes)){
        sync->setMonitored(nodes);
    }else{
        std::string msg;
        if(nh_priv.getParam("heartbeat/msg", msg)){
            can::Frame f = can::toframe(msg);
            if(f.isValid() && (f.id & ~canopen::BCMsync::ALL_NODES_MASK) == canopen::BCMsync::HEARTBEAT_ID){
                nodes.push_back(f.id & canopen::BCMsync::ALL_NODES_MASK);
            }
        }
        sync_nh.getParam("ignored_nodes",nodes);
        sync->setIgnored(nodes);
    }

    canopen::LayerStack stack("SyncNodeLayer");

    stack.add(boost::make_shared<canopen::CANLayer>(driver, can_device, false));
    stack.add(sync);

    diagnostic_updater::Updater diag_updater(nh, nh_priv);
    diag_updater.setHardwareID(nh_priv.param("hardware_id", std::string("none")));
    diag_updater.add("sync", boost::bind(report_diagnostics,boost::ref(stack), _1));

    ros::Timer diag_timer = nh.createTimer(ros::Duration(diag_updater.getPeriod()/2.0),boost::bind(&diagnostic_updater::Updater::update, &diag_updater));

    ros::spin();
    return 0;
}
