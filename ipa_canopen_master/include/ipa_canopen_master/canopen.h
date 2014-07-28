#ifndef H_IPA_CANOPEN
#define H_IPA_CANOPEN

#include <ipa_can_interface/interface.h>
#include <ipa_can_interface/dispatcher.h>
#include "objdict.h"
#include "timer.h"
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
    : interface_(interface), storage_(boost::make_shared<ObjectStorage>(dict, node_id, ObjectStorage::ReadDelegate(this, &SDOClient::read), ObjectStorage::WriteDelegate(this, &SDOClient::write)))
    {
        init();
    }
};

class PDOMapper{
    boost::mutex mutex_;
    
    class Buffer{
    public:
        bool read(uint8_t* b, const size_t len);
        void write(const uint8_t* b, const size_t len);
        void read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data);
        void write(const ipa_canopen::ObjectDict::Entry &, const std::string &data);
        const size_t size;
        Buffer(const size_t sz) : size(sz), dirty(false), empty(true), buffer(sz) {}
        
    private:
        boost::mutex mutex;
        boost::condition_variable cond;
        bool dirty;
        bool empty;
        std::vector<char> buffer;
    };
    
    class PDO {
    protected:
        void parse_and_set_mapping(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index, const bool &read, const bool &write);
        ipa_can::Frame frame;
        uint8_t transmission_type;
        std::vector< boost::shared_ptr<Buffer> >buffers;
    };
    
    struct TPDO: public PDO{
        void sync();
        static boost::shared_ptr<TPDO> create(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
            boost::shared_ptr<TPDO> tpdo(new TPDO(interface));
            if(!tpdo->init(storage, com_index, map_index))
                tpdo.reset();
            return tpdo;
        }
    private:
        TPDO(const boost::shared_ptr<ipa_can::Interface> interface) : interface_(interface){}
        bool init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index);
        const boost::shared_ptr<ipa_can::Interface> interface_;
        boost::mutex mutex;
    };
    
    struct RPDO : public PDO{
        void sync();
        static boost::shared_ptr<RPDO> create(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
            boost::shared_ptr<RPDO> rpdo(new RPDO(interface));
            if(!rpdo->init(storage, com_index, map_index))
                rpdo.reset();
            return rpdo;
        }
    private:
        bool init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index);
        RPDO(const boost::shared_ptr<ipa_can::Interface> interface) : interface_(interface), timeout(-1) {}
        boost::mutex mutex;
        const boost::shared_ptr<ipa_can::Interface> interface_;
        
        ipa_can::Interface::FrameListener::Ptr listener_;
        void handleFrame(const ipa_can::Frame & msg);
        int timeout;
    };
    
    boost::unordered_set< boost::shared_ptr<RPDO> > rpdos_;
    boost::unordered_set< boost::shared_ptr<TPDO> > tpdos_;
    
    const boost::shared_ptr<ipa_can::Interface> interface_;

public:
    PDOMapper(const boost::shared_ptr<ipa_can::Interface> interface);
    void sync(const uint8_t &counter);
    void init(const boost::shared_ptr<ObjectStorage> storage);
};



class SyncProvider{
public:
    typedef fastdelegate::FastDelegate1<const uint8_t&> SyncDelegate;
    typedef ipa_can::Listener<const SyncDelegate, const uint8_t&> SyncListener;
    
    SyncListener::Ptr add(const SyncDelegate & s);
    SyncProvider(boost::shared_ptr<ipa_can::Interface> interface,const ipa_can::Header &h, const boost::posix_time::time_duration &t, const uint8_t &overflow, bool loopback = true);
    const boost::posix_time::time_duration period;
    const uint8_t overflow_;
private:
    boost::shared_ptr<ipa_can::Interface> interface_;
    ipa_can::Frame msg_;
    ipa_can::SimpleDispatcher<SyncListener> syncables_;
    Timer timer_;
    boost::posix_time::time_duration timeout;
    boost::posix_time::time_duration max_timeout;
    boost::posix_time::time_duration track_timeout;
    ipa_can::Interface::FrameListener::Ptr loop_listener_;
    boost::mutex mutex_;
    
    void handleFrame(const ipa_can::Frame & msg);

    bool checkSync();
    
    bool sync_counter();
    bool sync_nocounter();
};

class Node{
public:
    enum State{
        Unknown = 255, BootUp = 0, Stopped = 4, Operational = 5 , PreOperational = 127
    };
    const uint8_t node_id_;
    Node(const boost::shared_ptr<ipa_can::Interface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id, const boost::shared_ptr<SyncProvider> sync = boost::shared_ptr<SyncProvider>());
    
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
    const boost::shared_ptr<SyncProvider> sync_;
    SyncProvider::SyncListener::Ptr sync_listener_;
    ipa_can::Interface::FrameListener::Ptr nmt_listener_;
    
    ObjectStorage::Entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> heartbeat_;

    
    void handleNMT(const ipa_can::Frame & msg);
    void switchState(const uint8_t &s);

    State state_;
    SDOClient sdo_;
    //EMCYHandler emcy;
    PDOMapper pdo_;
};

} // ipa_canopen
#endif // !H_IPA_CANOPEN
