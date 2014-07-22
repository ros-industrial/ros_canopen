#ifndef H_IPA_CANOPEN
#define H_IPA_CANOPEN

#include <ipa_can_interface/interface.h>
#include "objdict.h"
#include <stdexcept>
#include <boost/thread/condition_variable.hpp>

namespace ipa_canopen{

template<typename T> struct FrameOverlay: public ipa_can::Frame{
    T &data;
    FrameOverlay(const Header &h) : ipa_can::Frame(h,sizeof(T)), data(*(T*) ipa_can::Frame::data.c_array()) {
        ipa_can::Frame::data.fill(0);
    }
    FrameOverlay(const ipa_can::Frame &f) : ipa_can::Frame(f), data(*(T*) ipa_can::Frame::data.c_array()) { }
};

class SDOClient{
    
    ipa_can::Interface::FrameListener::Ptr listener_;
    ipa_can::Header client_id;
    
    boost::timed_mutex mutex;
    boost::mutex cond_mutex;
    boost::condition_variable cond;
    bool success;
    
    void handleFrame(const ipa_can::Frame & msg);
    
    std::string buffer;
    size_t offset;
    size_t total;
    ipa_can::Frame last_msg;
    const ipa_canopen::ObjectDict::Entry * current_entry;
    
    void wait_for_response();
    void abort(uint32_t reason);

    const boost::shared_ptr<ipa_can::Interface> interface_;
protected:
    void read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data);
    void write(const ipa_canopen::ObjectDict::Entry &entry, const std::string &data);
public:
    const boost::shared_ptr<ObjectStorage> storage_;
    
    void init();
    
    SDOClient(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id)
    : storage_(boost::make_shared<ObjectStorage>(dict, node_id, ObjectStorage::ReadDelegate(this, &SDOClient::read), ObjectStorage::WriteDelegate(this, &SDOClient::write))), interface_(interface)
    {
        init();
    }
};

class RPDOHandler{
    ipa_can::Interface::FrameListener::Ptr listener_;
    void handleFrame(const ipa_can::Frame & msg);
    std::vector<boost::shared_ptr<ObjectStorage::Buffer> > buffers_;
public:
    RPDOHandler(const boost::shared_ptr<ipa_can::Interface> interface, const ipa_can::Header &h, const std::vector<boost::shared_ptr<ObjectStorage::Buffer> > &buffers);
};

class PDOMapper{
    
    struct TPDO{
       ipa_can::Frame frame;
       std::vector< boost::shared_ptr<ObjectStorage::Buffer> >buffers;
    };
    struct RPDO{
       uint8_t transmission_type;
       bool rtr;
       boost::shared_ptr<RPDOHandler> handler;
    };
    boost::unordered_map<ipa_can::Header, TPDO > tpdos_;
    boost::unordered_map<ipa_can::Header, RPDO > rpdos_;
    
    const boost::shared_ptr<ipa_can::Interface> interface_;

    void setup_rpdos(const boost::shared_ptr<ObjectStorage> storage);
    void setup_tpdos(const boost::shared_ptr<ObjectStorage> storage);
protected:
    void read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data);
    void write(const ipa_canopen::ObjectDict::Entry &entry, const std::string &data);
public:
    PDOMapper(const boost::shared_ptr<ipa_can::Interface> interface);
    void sync();
    void init(const boost::shared_ptr<ObjectStorage> storage);
};

class Node{
public:
    enum State{
        Unknown = 255, BootUp = 0, Stopped = 4, Operational = 5 , PreOperational = 127
    };
    const uint8_t node_id_;
    Node(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id);
    
    const State& getState();
    void enterState(const State &s);
    
    const boost::shared_ptr<ObjectStorage> getStorage() { return sdo_.storage_; }
    
    void start();
    void stop();
    void reset();
    void reset_com();
    void prepare();
    
private:
    template<typename T> void wait_for(const State &s, const T &timeout);
    
    boost::timed_mutex mutex;
    boost::mutex cond_mutex;
    boost::condition_variable cond;
    
    const boost::shared_ptr<ipa_can::Interface> interface_;
    ipa_can::Interface::FrameListener::Ptr nmt_listener_;
    ipa_can::Header nmt_id;
    
    ObjectStorage::Entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> heartbeat_;

    
    void handleNMT(const ipa_can::Frame & msg);

    State state_;
    SDOClient sdo_;
    //EMCYHandler emcy;
    PDOMapper pdo_;
};

}; // ipa_canopen
#endif // !H_IPA_CANOPEN
