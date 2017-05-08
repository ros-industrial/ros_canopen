#ifndef H_CANOPEN
#define H_CANOPEN

#include <socketcan_interface/interface.h>
#include <socketcan_interface/dispatcher.h>
#include <socketcan_interface/reader.h>
#include "exceptions.h"
#include "layer.h"
#include "objdict.h"
#include "timer.h"
#include <stdexcept>
#include <boost/thread/condition_variable.hpp>
#include <boost/chrono/system_clocks.hpp>
#include <boost/lexical_cast.hpp>

namespace canopen{

typedef boost::chrono::high_resolution_clock::time_point time_point;
typedef boost::chrono::high_resolution_clock::duration time_duration;
inline time_point get_abs_time(const time_duration& timeout) { return boost::chrono::high_resolution_clock::now() + timeout; }
inline time_point get_abs_time() { return boost::chrono::high_resolution_clock::now(); }


template<typename T> struct FrameOverlay: public can::Frame{
    T &data;
    FrameOverlay(const Header &h) : can::Frame(h,sizeof(T)), data(*(T*) can::Frame::data.c_array()) {
        can::Frame::data.fill(0);
    }
    FrameOverlay(const can::Frame &f) : can::Frame(f), data(*(T*) can::Frame::data.c_array()) { }
};

class SDOClient{

    can::Header client_id;

    boost::timed_mutex mutex;

    can::BufferedReader reader_;
    bool processFrame(const can::Frame & msg);

    String buffer;
    size_t offset;
    size_t total;
    bool done;
    can::Frame last_msg;
    const canopen::ObjectDict::Entry * current_entry;

    void transmitAndWait(const canopen::ObjectDict::Entry &entry, const String &data, String *result);
    void abort(uint32_t reason);

    const boost::shared_ptr<can::CommInterface> interface_;
protected:
    void read(const canopen::ObjectDict::Entry &entry, String &data);
    void write(const canopen::ObjectDict::Entry &entry, const String &data);
public:
    const boost::shared_ptr<ObjectStorage> storage_;

    void init();

    SDOClient(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id)
    : interface_(interface), storage_(boost::make_shared<ObjectStorage>(dict, node_id, ObjectStorage::ReadDelegate(this, &SDOClient::read), ObjectStorage::WriteDelegate(this, &SDOClient::write))), reader_(false, 1)
    {
    }
};

class PDOMapper{
    boost::mutex mutex_;

    class Buffer{
    public:
        bool read(uint8_t* b, const size_t len);
        void write(const uint8_t* b, const size_t len);
        void read(const canopen::ObjectDict::Entry &entry, String &data);
        void write(const canopen::ObjectDict::Entry &, const String &data);
        void clean() { dirty = false; }
        const size_t size;
        Buffer(const size_t sz) : size(sz), dirty(false), empty(true), buffer(sz) {}

    private:
        boost::mutex mutex;
        bool dirty;
        bool empty;
        std::vector<char> buffer;
    };

    class PDO {
    protected:
        void parse_and_set_mapping(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index, const bool &read, const bool &write);
        can::Frame frame;
        uint8_t transmission_type;
        std::vector< boost::shared_ptr<Buffer> >buffers;
    };

    struct TPDO: public PDO{
        void sync();
        static boost::shared_ptr<TPDO> create(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
            boost::shared_ptr<TPDO> tpdo(new TPDO(interface));
            if(!tpdo->init(storage, com_index, map_index))
                tpdo.reset();
            return tpdo;
        }
    private:
        TPDO(const boost::shared_ptr<can::CommInterface> interface) : interface_(interface){}
        bool init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index);
        const boost::shared_ptr<can::CommInterface> interface_;
        boost::mutex mutex;
    };

    struct RPDO : public PDO{
        void sync(LayerStatus &status);
        static boost::shared_ptr<RPDO> create(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
            boost::shared_ptr<RPDO> rpdo(new RPDO(interface));
            if(!rpdo->init(storage, com_index, map_index))
                rpdo.reset();
            return rpdo;
        }
    private:
        bool init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index);
        RPDO(const boost::shared_ptr<can::CommInterface> interface) : interface_(interface), timeout(-1) {}
        boost::mutex mutex;
        const boost::shared_ptr<can::CommInterface> interface_;

        can::CommInterface::FrameListener::Ptr listener_;
        void handleFrame(const can::Frame & msg);
        int timeout;
    };

    boost::unordered_set< boost::shared_ptr<RPDO> > rpdos_;
    boost::unordered_set< boost::shared_ptr<TPDO> > tpdos_;

    const boost::shared_ptr<can::CommInterface> interface_;

public:
    PDOMapper(const boost::shared_ptr<can::CommInterface> interface);
    void read(LayerStatus &status);
    bool write();
    bool init(const boost::shared_ptr<ObjectStorage> storage, LayerStatus &status);
};

class EMCYHandler : public Layer {
    boost::atomic<bool> has_error_;
    ObjectStorage::Entry<uint8_t> error_register_;
    ObjectStorage::Entry<uint8_t> num_errors_;
    can::CommInterface::FrameListener::Ptr emcy_listener_;
    void handleEMCY(const can::Frame & msg);
    const boost::shared_ptr<ObjectStorage> storage_;

    virtual void handleDiag(LayerReport &report);

    virtual void handleInit(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);

public:
    EMCYHandler(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectStorage> storage);
    void resetErrors(LayerStatus &status);
};

struct SyncProperties{
    const can::Header header_;
    const uint16_t period_ms_;
    const uint8_t overflow_;
    SyncProperties(const can::Header &h, const uint16_t  &p, const uint8_t &o) : header_(h), period_ms_(p), overflow_(o) {}
    bool operator==(const SyncProperties &p) const { return p.header_ == (int) header_ && p.overflow_ == overflow_ && p.period_ms_ == period_ms_; }

};

class SyncCounter {
public:
    const SyncProperties properties;
    SyncCounter(const SyncProperties &p) : properties(p) {}
    virtual void addNode(void * const ptr)  = 0;
    virtual  void removeNode(void * const ptr) = 0;
    virtual ~SyncCounter() {}
};

class Node : public Layer{
public:
    enum State{
        Unknown = 255, BootUp = 0, Stopped = 4, Operational = 5 , PreOperational = 127
    };
    const uint8_t node_id_;
    Node(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectDict> dict, uint8_t node_id, const boost::shared_ptr<SyncCounter> sync = boost::shared_ptr<SyncCounter>());

    const State getState();
    void enterState(const State &s);

    const boost::shared_ptr<ObjectStorage> getStorage() { return sdo_.storage_; }

    bool start();
    bool stop();
    bool reset();
    bool reset_com();
    bool prepare();

    typedef fastdelegate::FastDelegate1<const State&> StateDelegate;
    typedef can::Listener<const StateDelegate, const State&> StateListener;

    StateListener::Ptr addStateListener(const StateDelegate & s){
        return state_dispatcher_.createListener(s);
    }

    template<typename T> T get(const ObjectDict::Key& k){
        return getStorage()->entry<T>(k).get();
    }

private:
    virtual void handleDiag(LayerReport &report);

    virtual void handleInit(LayerStatus &status);
    virtual void handleRecover(LayerStatus &status);
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state);
    virtual void handleHalt(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status);

    template<typename T> int wait_for(const State &s, const T &timeout);

    boost::timed_mutex mutex;
    boost::mutex cond_mutex;
    boost::condition_variable cond;

    const boost::shared_ptr<can::CommInterface> interface_;
    const boost::shared_ptr<SyncCounter> sync_;
    can::CommInterface::FrameListener::Ptr nmt_listener_;

    ObjectStorage::Entry<ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> heartbeat_;

    can::SimpleDispatcher<StateListener> state_dispatcher_;

    void handleNMT(const can::Frame & msg);
    void switchState(const uint8_t &s);

    State state_;
    SDOClient sdo_;
    PDOMapper pdo_;

    boost::chrono::high_resolution_clock::time_point heartbeat_timeout_;
    uint16_t getHeartbeatInterval() { return heartbeat_.valid()?heartbeat_.get_cached() : 0; }
    void setHeartbeatInterval() { if(heartbeat_.valid()) heartbeat_.set(heartbeat_.desc().value().get<uint16_t>()); }
    bool checkHeartbeat();
};

template<typename T> class Chain{
protected:
    std::vector<boost::shared_ptr<T> > elements;
public:
    void call(void (T::*func)(void)){
        typename std::vector<boost::shared_ptr<T> >::iterator it = elements.begin();
        while(it != elements.end()){
            ((**it).*func)();
            ++it;
        }
    }
    template<typename V> void call(void (T::*func)(const V&), const std::vector<V> &vs){
        typename std::vector<boost::shared_ptr<T> >::iterator it = elements.begin();
        typename std::vector<V>::const_iterator it_v = vs.begin();
        while(it_v != vs.end() &&  it != elements.end()){
            ((**it).*func)(*it_v);
            ++it; ++it_v;
        }
    }
    template<typename V> void call(void (T::*func)(V&), std::vector<V> &vs){
        vs.resize(elements.size());

        typename std::vector<boost::shared_ptr<T> >::iterator it = elements.begin();
        typename std::vector<V>::iterator it_v = vs.begin();
        while(it_v != vs.end() &&  it != elements.end()){
            ((**it).*func)(*it_v);
            ++it; ++it_v;
        }
    }
    void add(boost::shared_ptr<T> t){
        elements.push_back(t);
    }
};

template<typename T> class NodeChain: public Chain<T>{
public:
    const std::vector<boost::shared_ptr<T> >& getElements() { return Chain<T>::elements; }
    void start() { this->call(&T::start); }
    void stop() { this->call(&T::stop); }
    void reset() { this->call(&T::reset); }
    void reset_com() { this->call(&T::reset_com); }
    void prepare() { this->call(&T::prepare); }
};

class SyncLayer: public Layer, public SyncCounter{
public:
    SyncLayer(const SyncProperties &p) : Layer("Sync layer"), SyncCounter(p) {}
};

class Master: boost::noncopyable {
public:
    virtual boost::shared_ptr<SyncLayer> getSync(const SyncProperties &properties) = 0;
    virtual ~Master() {}

    class Allocator {
    public:
        virtual boost::shared_ptr<Master> allocate(const std::string &name, boost::shared_ptr<can::CommInterface> interface) = 0;
        virtual ~Allocator() {}
    };
};

class Settings
{
public:
    template <typename T> T get_optional(const std::string &n, const T& def) const {
        std::string repr;
        if(!getRepr(n, repr)){
            return def;
        }
        return boost::lexical_cast<T>(repr);
    }
    template <typename T> bool get(const std::string &n, T& val) const {
        std::string repr;
        if(!getRepr(n, repr)) return false;
        val =  boost::lexical_cast<T>(repr);
        return true;
    }
    virtual ~Settings() {}
private:
    virtual bool getRepr(const std::string &n, std::string & repr) const = 0;
};


/*template<typename InterfaceType, typename MasterType, typename NodeType> class Bus: boost::noncopyable{
    boost::weak_ptr <InterfaceType> weak_interface_;
    boost::weak_ptr <MasterType> weak_master_;

    const String device_;
    const unsigned int bitrate_;
public:
    Bus(const String &device, unsigned int bitrate) : device_(device), bitrate_(bitrate) {}
    boost::shared_ptr<InterfaceType> getInterface(){
        boost::shared_ptr<InterfaceType> interface = weak_interface_.lock();
        if(!interface){
            weak_interface_ = interface = boost::make_shared<InterfaceType>();
            interface_
        }
        return interface;
    }
    boost::shared_ptr<MasterType> getMaster(){
        boost::shared_ptr<MasterType> master = weak_master_.lock();
        if(!master){
            boost::shared_ptr<InterfaceType> interface = getInterface();
            if(interface) weak_master_ = master = boost::make_shared<MasterType>(interface);
        }
        return master;
    }
};*/

} // canopen
#endif // !H_CANOPEN
