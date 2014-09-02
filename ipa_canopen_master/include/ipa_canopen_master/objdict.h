#ifndef H_IPA_OBJDICT
#define H_IPA_OBJDICT

#include <ipa_can_interface/FastDelegate.h>
#include <boost/unordered_map.hpp>    
#include <boost/unordered_set.hpp>    
#include <boost/thread/mutex.hpp>    
#include <boost/make_shared.hpp>
#include <typeinfo> 
#include <vector>

namespace ipa_canopen{
class AccessException {};
class TimeoutException {};
class ParseException {};

class TypeGuard{
    const std::type_info& (*get_type)();
    size_t type_size;

    template<typename T> class TypeInfo{
    public:
        static const std::type_info& id() { return typeid(T); }
    };
    TypeGuard(const std::type_info& (*ti)(), const size_t s): get_type(ti), type_size(s) {}
public:

    template<typename T> bool is_type() const {
        return valid() && get_type() == typeid(T);
    }
    
    bool operator==(const TypeGuard &other) const {
        return valid() && other.valid() && (get_type() == other.get_type());
    }
    
    TypeGuard(): get_type(0), type_size(0) {}
    bool valid() const { return get_type != 0; }
    size_t get_size() const { return type_size; }
    template<typename T> static TypeGuard create() { return TypeGuard(TypeInfo<T>::id, sizeof(T)); }
};

class HoldAny{
    std::string buffer;
    TypeGuard type_guard;
    bool empty;
public:
    HoldAny() : empty(true) {}
    
    const TypeGuard& type() const{ return type_guard; }
    
    template<typename T> HoldAny(const T &t) : type_guard(TypeGuard::create<T>()), empty(false){
        buffer.resize(sizeof(T));
        *(T*)&buffer[0] = t;
    }
    HoldAny(const std::string &t): type_guard(TypeGuard::create<std::string>()), empty(false){
        if(!type_guard.is_type<std::string>()){
            throw std::bad_cast();
        }
        buffer = t;
    }
    HoldAny(const TypeGuard &t): type_guard(t), empty(true){ }

    bool is_empty() const { return empty; }
    
    const std::string data() const { 
        if(empty){
            throw std::length_error("buffer empty");
        }        
        return buffer;
    }

    template<typename T> const T & get() const{
        if(!type_guard.is_type<T>()){
            throw std::bad_cast();
        }else if(empty){
            throw std::length_error("buffer empty");
        }
        return *(T*)&buffer[0];
    }
};

template<> const std::string & HoldAny::get() const;

struct DeviceInfo{
    std::string vendor_name;
    uint32_t vendor_number;
    std::string product_name;
    uint32_t product_number;
    uint32_t revision_number;
    std::string order_code;
    boost::unordered_set<uint32_t> baudrates;
    bool simple_boot_up_master;
    bool simple_boot_up_slave;
    uint8_t granularity;
    bool dynamic_channels_supported;
    bool group_messaging;
    uint16_t nr_of_rx_pdo;
    uint16_t nr_of_tx_pdo;
    bool lss_supported;
    boost::unordered_set<uint16_t> dummy_usage;
};

class ObjectDict{
protected:
public:
    class Key{
    public:
        const size_t hash;
        Key(const uint16_t i) : hash((i<<16)| 0xFFFF) {}
        Key(const uint16_t i, const uint8_t s): hash((i<<16)| s) {}
        bool operator==(const Key &other) const { return hash == other.hash; }
    };
    enum Code{
        NULL_DATA = 0x00,
        DOMAIN_DATA = 0x02,
        DEFTYPE = 0x05,
        DEFSTRUCT = 0x06,
        VAR = 0x07,
        ARRAY = 0x08,
        RECORD = 0x09
    };
    enum DataTypes{
        DEFTYPE_INTEGER8 = 0x0002,
        DEFTYPE_INTEGER16 = 0x0003,
        DEFTYPE_INTEGER32 = 0x0004,
        DEFTYPE_UNSIGNED8 = 0x0005,
        DEFTYPE_UNSIGNED16 = 0x0006,
        DEFTYPE_UNSIGNED32 = 0x0007,
        DEFTYPE_REAL32 = 0x0008,
        DEFTYPE_VISIBLE_STRING = 0x0009,
        DEFTYPE_OCTET_STRING = 0x000A,
        DEFTYPE_UNICODE_STRING = 0x000B,
        DEFTYPE_DOMAIN = 0x000F,
        DEFTYPE_REAL64 = 0x0010,
        DEFTYPE_INTEGER64 = 0x0015,
        DEFTYPE_UNSIGNED64 = 0x001B
    };
    struct Entry{
        Code obj_code;
        uint16_t index;
        uint8_t sub_index;
        uint16_t data_type;
        bool readable;
        bool writable;
        bool mappable;
        std::string desc;
        HoldAny def_val;
        HoldAny init_val;
        
        Entry() {}
        
        Entry(const Code c, const uint16_t i,  const uint16_t t, const std::string & d, const bool r = true, const bool w = true, bool m = false, const HoldAny def = HoldAny(), const HoldAny init = HoldAny()):
        obj_code(c), index(i), sub_index(0),data_type(t),readable(r), writable(w), mappable(m), desc(d), def_val(def), init_val(init) {}
        
        Entry(const uint16_t i, const uint8_t s, const uint16_t t, const std::string & d, const bool r = true, const bool w = true, bool m = false, const HoldAny def = HoldAny(), const HoldAny init = HoldAny()):
        obj_code(VAR), index(i), sub_index(s),data_type(t),readable(r), writable(w), mappable(m), desc(d), def_val(def), init_val(init) {}
        
        const HoldAny & value() const { return !init_val.is_empty() ? init_val : def_val; }
            
    };
    const Entry& operator()(uint16_t i) const{
        return *dict_.at(Key(i));
    }
    const Entry& operator()(uint16_t i, uint8_t s) const{
        return *dict_.at(Key(i,s));
    }
    const boost::shared_ptr<const Entry>& get(const Key &k) const{
        return dict_.at(k);
    }
    bool insert(bool is_sub, boost::shared_ptr<const Entry> e){
        std::pair<boost::unordered_map<Key, boost::shared_ptr<const Entry> >::iterator, bool>  res = dict_.insert(std::make_pair(is_sub?Key(e->index,e->sub_index):Key(e->index),e));
        return res.second;
    }
    bool iterate(boost::unordered_map<Key, boost::shared_ptr<const Entry> >::const_iterator &it) const;
    static boost::shared_ptr<ObjectDict> fromFile(const std::string &path);
    
    const DeviceInfo device_info;
    
    ObjectDict(const DeviceInfo &info): device_info(info) {}
protected:
    boost::unordered_map<Key, boost::shared_ptr<const Entry> > dict_;
};

std::size_t hash_value(ObjectDict::Key const& k);

template<typename T> class NodeIdOffset{
    T offset;
    T (*adder)(const uint8_t &, const T &);
    
    static T add(const uint8_t &u, const T &t) {
        return u+t;
    }
public:
    NodeIdOffset(const T &t): offset(t), adder(add) {}
    
    static const T apply(const HoldAny &val, const uint8_t &u){
        if(!val.is_empty()){
            if(TypeGuard::create<T>() ==  val.type() ){
                return val.get<T>();
            }else{
                const NodeIdOffset<T> &no = val.get< NodeIdOffset<T> >();                
                return no.adder(u, no.offset);
            }
        }else{
            throw std::bad_cast();
        }
        
    }
};

template<typename T> std::ostream& operator<<(std::ostream& stream, const NodeIdOffset<T> &n) {
    //stream << "Offset: " << n.apply(0);
    return stream;
 }

class ObjectStorage{
public:
    typedef fastdelegate::FastDelegate2<const ObjectDict::Entry&, std::string &> ReadDelegate;
    typedef fastdelegate::FastDelegate2<const ObjectDict::Entry&, const std::string &> WriteDelegate;
    
protected:
    class Data: boost::noncopyable{
        boost::mutex mutex;
        std::string buffer;
        bool valid;

        ReadDelegate read_delegate;
        WriteDelegate write_delegate;
        
        template <typename T> T & access(){
            if(!valid){
                throw std::length_error("buffer not valid");
            }
            return *(T*)&buffer[0];
        }
        template <typename T> T & allocate(){
            if(!valid){
                buffer.resize(sizeof(T));
                valid = true;
            }
            return access<T>();
        }
    public:
        const TypeGuard type_guard;
        const boost::shared_ptr<const ObjectDict::Entry> entry;
        size_t size() { boost::mutex::scoped_lock lock(mutex); return buffer.size(); }
        
        template<typename T> Data(const boost::shared_ptr<const ObjectDict::Entry> &e, const T &val, const ReadDelegate &r, const WriteDelegate &w)
        : valid(false), read_delegate(r), write_delegate(w), type_guard(TypeGuard::create<T>()), entry(e){
            assert(!r.empty());
            assert(!w.empty());
            assert(e);
            allocate<T>() = val;
        }
        Data(const boost::shared_ptr<const ObjectDict::Entry> &e, const TypeGuard &t, const ReadDelegate &r, const WriteDelegate &w)
        : valid(false), read_delegate(r), write_delegate(w), type_guard(t), entry(e){
            assert(!r.empty());
            assert(!w.empty());
            assert(e);
            assert(t.valid());
            buffer.resize(t.get_size());
        }
        void set_delegates(const ReadDelegate &r, const WriteDelegate &w){
            boost::mutex::scoped_lock lock(mutex);
            if(r) read_delegate = r;
            if(w) write_delegate = w;
        }
        template<typename T> const T get_cached() {
            boost::mutex::scoped_lock lock(mutex);
            
            if(!entry->readable){
                if(entry->writable){
                    throw AccessException();
                }
            }
            if(valid){
                return access<T>();
            }else{
                throw AccessException();
            }
        }

        template<typename T> const T get() {
            boost::mutex::scoped_lock lock(mutex);
            
            if(!entry->readable){
                if(entry->writable){
                    throw AccessException();
                }
                return get_cached<T>();
            }
            
            allocate<T>();
            read_delegate(*entry, buffer);
            return access<T>();
        }
        template<typename T>  void set(const T &val) {
            boost::mutex::scoped_lock lock(mutex);
            
            if(!entry->writable){
                if(access<T>() != val){
                    throw AccessException();
                }
            }else{
                allocate<T>() = val;
                write_delegate(*entry, buffer);
            }
        }
        void init();
        void reset();

    };        
        
public:
    template<const uint16_t dt> struct DataType{
        typedef void type;
    };
    
    template<typename T> class Entry{
        boost::shared_ptr<Data> data;
    public:
        typedef T type;
        const T get() {
            if(!data) throw AccessException();

            return data->get<T>();
        }        
        const T get_cached() {
            if(!data) throw AccessException();

            return data->get_cached<T>();
        }        
        void set(const T &val) {
            if(!data) throw AccessException();
            data->set(val);
        }
        
        Entry() {}
        Entry(boost::shared_ptr<Data> &d)
        : data(d){
            assert(d);
        }
        const ObjectDict::Entry & desc() const{
            return *(data->entry);
        }
    };
    
    void reset();
    
protected:
    boost::unordered_map<ObjectDict::Key, boost::shared_ptr<Data> > storage_;
    boost::mutex mutex_;
    
    template<typename T> Entry<T> entry(const ObjectDict::Key &key){
        boost::mutex::scoped_lock lock(mutex_);
        
        boost::unordered_map<ObjectDict::Key, boost::shared_ptr<Data> >::iterator it = storage_.find(key);
        
        if(it == storage_.end()){
            const boost::shared_ptr<const ObjectDict::Entry> e = dict_->get(key);
            
            boost::shared_ptr<Data> data;
            TypeGuard type = TypeGuard::create<T>();
    
            if(!e->def_val.is_empty()){
                T val = NodeIdOffset<T>::apply(e->def_val, node_id_);
                data = boost::make_shared<Data>(e,val, read_delegate_, write_delegate_);
            }else{
                if(!e->def_val.type().valid() ||  e->def_val.type() == type) {
                    data = boost::make_shared<Data>(e,type, read_delegate_, write_delegate_);
                }else{
                    throw std::bad_cast();
                }
            }
            
            std::pair<boost::unordered_map<ObjectDict::Key, boost::shared_ptr<Data> >::iterator, bool>  ok = storage_.insert(std::make_pair(key, data));
            it = ok.first;
        }
        
        if(!it->second->type_guard.is_type<T>()){
            throw std::bad_cast();
        }
        return Entry<T>(it->second);
    }
    void init_nolock(const ObjectDict::Key &key, const boost::shared_ptr<const ObjectDict::Entry> &entry);
    
    ReadDelegate read_delegate_;
    WriteDelegate write_delegate_;
    size_t map(const boost::shared_ptr<const ObjectDict::Entry> &e, const ObjectDict::Key &key, const ReadDelegate & read_delegate, const WriteDelegate & write_delegate);
public:
    
    size_t map(uint16_t index, uint8_t sub_index, const ReadDelegate & read_delegate, const WriteDelegate & write_delegate);
    
    template<typename T> Entry<T> entry(uint16_t index){
        return entry<T>(ObjectDict::Key(index));
    }
    template<typename T> Entry<T> entry(uint16_t index, uint8_t sub_index){
        return entry<T>(ObjectDict::Key(index,sub_index));
    }
    
    template<typename T> void entry(Entry<T> &e, uint16_t index){
        e = entry<T>(ObjectDict::Key(index));
    }
     template<typename T> void entry(Entry<T> &e, uint16_t index, uint8_t sub_index){
        e = entry<T>(ObjectDict::Key(index,sub_index));
    }

    const boost::shared_ptr<const ObjectDict> dict_;
    const uint8_t node_id_;
    
    ObjectStorage(boost::shared_ptr<const ObjectDict> dict, uint8_t node_id, ReadDelegate read_delegate, WriteDelegate write_delegate);
    
    void init(const ObjectDict::Key &key);
    void init_all();
};

template<> std::string & ObjectStorage::Data::access();
template<> std::string & ObjectStorage::Data::allocate();

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER8> { typedef int8_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER16> { typedef int16_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER32> { typedef int32_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER64> { typedef int64_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED8> { typedef uint8_t type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16> { typedef uint16_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED32> { typedef uint32_t type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED64> { typedef uint64_t type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL32> { typedef float type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_REAL64> { typedef double type;};

template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_VISIBLE_STRING> { typedef std::string type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_OCTET_STRING> { typedef std::string type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_UNICODE_STRING> { typedef std::string type;};
template<> struct ObjectStorage::DataType<ObjectDict::DEFTYPE_DOMAIN> { typedef std::string type;};

template<typename T, typename R> static R *branch_type(const uint16_t data_type){
    switch(ObjectDict::DataTypes(data_type)){
        case ObjectDict::DEFTYPE_INTEGER8: return T::template func< ObjectDict::DEFTYPE_INTEGER8 >;
        case ObjectDict::DEFTYPE_INTEGER16: return T::template func< ObjectDict::DEFTYPE_INTEGER16 >;
        case ObjectDict::DEFTYPE_INTEGER32: return T::template func< ObjectDict::DEFTYPE_INTEGER32 >;
        case ObjectDict::DEFTYPE_INTEGER64: return T::template func< ObjectDict::DEFTYPE_INTEGER64 >;
            
        case ObjectDict::DEFTYPE_UNSIGNED8: return T::template func< ObjectDict::DEFTYPE_UNSIGNED8 >;
        case ObjectDict::DEFTYPE_UNSIGNED16: return T::template func< ObjectDict::DEFTYPE_UNSIGNED16 >;
        case ObjectDict::DEFTYPE_UNSIGNED32: return T::template func< ObjectDict::DEFTYPE_UNSIGNED32 >;
        case ObjectDict::DEFTYPE_UNSIGNED64: return T::template func< ObjectDict::DEFTYPE_UNSIGNED64 >;
            
        case ObjectDict::DEFTYPE_REAL32: return T::template func< ObjectDict::DEFTYPE_REAL32 >;
        case ObjectDict::DEFTYPE_REAL64: return T::template func< ObjectDict::DEFTYPE_REAL64 >;

        case ObjectDict::DEFTYPE_VISIBLE_STRING: return T::template func< ObjectDict::DEFTYPE_VISIBLE_STRING >;
        case ObjectDict::DEFTYPE_OCTET_STRING: return T::template func< ObjectDict::DEFTYPE_OCTET_STRING >;
        case ObjectDict::DEFTYPE_UNICODE_STRING: return T::template func< ObjectDict::DEFTYPE_UNICODE_STRING >;
        case ObjectDict::DEFTYPE_DOMAIN: return T::template func< ObjectDict::DEFTYPE_DOMAIN >;
           
        default: return 0;
    }
}

} // ipa_canopen

#endif // !H_IPA_OBJDICT
