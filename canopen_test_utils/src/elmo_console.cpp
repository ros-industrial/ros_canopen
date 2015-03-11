#include <canopen_master/canopen.h>
#include <canopen_master/master.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <socketcan_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>

#include <boost/thread.hpp>

using namespace can;
using namespace canopen;

class ElmoNode: public Node
{
    static boost::shared_ptr<canopen::ObjectDict>  make_dict(){
        canopen::DeviceInfo info;
        info.nr_of_rx_pdo = 0;
        info.nr_of_tx_pdo = 0;

        boost::shared_ptr<canopen::ObjectDict>  dict = boost::make_shared<canopen::ObjectDict>(info);

        dict->insert(true, boost::make_shared<const canopen::ObjectDict::Entry>(0x1023, 1, ObjectDict::DEFTYPE_OCTET_STRING, "Command", false, true, false));
        dict->insert(true, boost::make_shared<const canopen::ObjectDict::Entry>(0x1023, 2, ObjectDict::DEFTYPE_UNSIGNED8, "Status", true, false, false));
        dict->insert(true, boost::make_shared<const canopen::ObjectDict::Entry>(0x1023, 3, ObjectDict::DEFTYPE_OCTET_STRING, "Reply", true, false, false));


        dict->insert(false, boost::make_shared<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x1024, ObjectDict::DEFTYPE_UNSIGNED8, "OS command mode", false, true, false));

        dict->insert(false, boost::make_shared<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x1001, ObjectDict::DEFTYPE_UNSIGNED8, "error register", true, false, false));

        dict->insert(false, boost::make_shared<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x1017, ObjectDict::DEFTYPE_UNSIGNED16, "producer heartbeat", true, true, false, HoldAny((uint16_t)0)));

        return dict;
    }

    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_OCTET_STRING>::type >  command_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED8>::type >  status_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_OCTET_STRING>::type >  reply_;

    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED8>::type >  mode_;

    template<typename T> bool try_set(T & entry, const typename T::type &value){
        try{
            entry.set(value);
        }
        catch(...){
            return false;
        }
        return true;
    }
public:

    ElmoNode(boost::shared_ptr<can::CommInterface> interface, const uint8_t &id) : Node (interface, make_dict(), id){
        getStorage()->entry(command_, 0x1023, 1);
        getStorage()->entry(status_, 0x1023, 2);
        getStorage()->entry(reply_, 0x1023, 3);

        getStorage()->entry(mode_, 0x1024);
    }

    uint8_t send_command(const std::string &s){
        if(!try_set(command_, String(s))) return 0;

        uint8_t status;

        do{
            status = status_.get();
        }while(status == 255);

        return status;
    }
    void get_response(std::string &s){
        String res;
        try{
            res = reply_.get();
            s = (const std::string&) res;
        }
        catch(const TimeoutException&){
            s = "<TIMEOUT>";
        }
    }

};



boost::shared_ptr<ThreadedSocketCANInterface> driver;
boost::shared_ptr<ElmoNode> node;
StateInterface::StateListener::Ptr state_printer;

void print_state(const State &s){
    boost::shared_ptr<ThreadedSocketCANInterface> d = driver;
    std::string msg;
    if(!s.error_code && !s.internal_error) return;
    
    if(d && !d->translateError(s.internal_error, msg)) msg  =  "Undefined";
    std::cerr << "device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")" << std::endl;
}

void shutdown(boost::shared_ptr<ThreadedSocketCANInterface> &driver, boost::shared_ptr<ElmoNode> &node, int code){
    state_printer.reset();

    LayerStatus s;
    if(node) node->shutdown(s);
    if(driver) driver->shutdown();
    exit(code);
}

void sigint_handler (int param)
{
  shutdown(driver, node, param);
}

int main(int argc, char *argv[]){

    if(argc <= 2){
        std::cerr << "usage: " << argv[0] << " device node_id [< INPUT] [> OUTPUT]" << std::endl;
       return -1;
    }

    bool tty = isatty(fileno(stdin));

    uint8_t nodeid = atoi(argv[2]);

    signal(SIGINT, sigint_handler);


    driver = boost::make_shared<ThreadedSocketCANInterface> ();
    state_printer = driver->createStateListener(print_state);

    if(!driver->init(argv[1],0)){
        std::cerr << "init failed" << std::endl;
        return -1;
    }

    sleep(1.0);

    node = boost::make_shared<ElmoNode>(driver, nodeid);

    LayerStatus status;
    try{
        node->init(status);
        if(!status.bounded<LayerStatus::Warn>()){
            std::cerr << status.reason() << std::endl;
            shutdown(driver,node,-1);
        }
    }
    catch( const canopen::Exception &e){
        std::cerr << boost::diagnostic_information(e) << std::endl;
        shutdown(driver,node,-1);
    }

    std::string res;
    std::string line;

    while(std::cin.good()){
        if(tty) std::cout << "> ";
        std::getline (std::cin, line);

        if(line.empty()) continue;
        
        if(!tty) std::cout << line <<  ": ";
            
        uint8_t status = node->send_command(line);
        if(status){
            node->get_response(res);
            std::cout << (status == 1 ? res : "<ERROR>")  << std::endl;
        }else{
            std::cout << "ERROR!";
            break;
        }
    }

    shutdown(driver,node,0);

    return 0;
}
