#include <canopen_master/canopen.h>

using namespace canopen;

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

class PDOid{
    const uint32_t value_;
public:
    static const unsigned int ID_MASK = (1u << 29)-1;
    static const unsigned int EXTENDED_MASK = (1u << 29);
    static const unsigned int NO_RTR_MASK = (1u << 30);
    static const unsigned int INVALID_MASK = (1u << 31);

    PDOid(const uint32_t &val)
    : value_(val)
    {}
    can::Header header(bool fill_rtr = false) const {
        return can::Header(value_ & ID_MASK, value_ & EXTENDED_MASK, fill_rtr && !(value_ & NO_RTR_MASK), false);
    }
    bool isInvalid() const { return value_ & INVALID_MASK; }
};

struct PDOmap{
    uint8_t length;
    uint8_t sub_index;
    uint16_t index;
    PDOmap(uint32_t val)
    : length(val & 0xFF),
      sub_index((val>>8) & 0xFF),
      index(val>>16)
    {}
};

#pragma pack(pop) /* pop previous alignment from stack */


const uint8_t SUB_COM_NUM = 0;
const uint8_t SUB_COM_COB_ID = 1;
const uint8_t SUB_COM_TRANSMISSION_TYPE = 2;
const uint8_t SUB_COM_RESERVED = 4;

const uint8_t SUB_MAP_NUM = 0;

const uint16_t RPDO_COM_BASE =0x1400;
const uint16_t RPDO_MAP_BASE =0x1600;
const uint16_t TPDO_COM_BASE =0x1800;
const uint16_t TPDO_MAP_BASE =0x1A00;

bool check_com_changed(const ObjectDict &dict, const uint16_t com_id){
    bool com_changed = false;
    
    // check if com parameter has to be set
    for(uint8_t sub = 0; sub <=6 ; ++sub){
        try{
            if(!dict(com_id,sub).init_val.is_empty()){
                com_changed = true;
                break;
            }
        }
        catch (std::out_of_range) {}
    }
    return com_changed;
}

bool check_map_changed(const uint8_t &num, const ObjectDict &dict, const uint16_t &map_index){
    bool map_changed = false;

    // check if mapping has to be set
    if(num <= 0x40){
        for(uint8_t sub = 1; sub <=num ; ++sub){
            try{
                if(!dict(map_index,sub).init_val.is_empty()){
                    map_changed = true;
                    break;
                }
            }
            catch (std::out_of_range) {}
        }
    }else{
        map_changed = dict( map_index ,0 ).init_val.is_empty();
    }
    return map_changed;
}
void PDOMapper::PDO::parse_and_set_mapping(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index, const bool &read, const bool &write){
                            
    const canopen::ObjectDict & dict = *storage->dict_;
    
    ObjectStorage::Entry<uint8_t> num_entry;
    storage->entry(num_entry, map_index, SUB_MAP_NUM);

    uint8_t map_num;
    
    try{
        map_num = num_entry.desc().value().get<uint8_t>();
    }catch(...){
        map_num = 0;
    }
    
    bool map_changed = check_map_changed(map_num, dict, map_index);
    
    // disable PDO if needed
    ObjectStorage::Entry<uint32_t> cob_id;
    storage->entry(cob_id, com_index, SUB_COM_COB_ID);
    
    bool com_changed = check_com_changed(dict, map_index);
    if((map_changed || com_changed) && cob_id.desc().writable){
        cob_id.set(cob_id.get() | PDOid::INVALID_MASK);
    }
    if(map_num > 0 && map_num <= 0x40){ // actual mapping 
        if(map_changed){
            num_entry.set(0);
        }
        
        frame.dlc = 0;
        for(uint8_t sub = 1; sub <=map_num; ++sub){
            ObjectStorage::Entry<uint32_t> mapentry;
            storage->entry(mapentry, map_index, sub);
            const HoldAny init = dict(map_index ,sub).init_val;
            if(!init.is_empty()) mapentry.set(init.get<uint32_t>());
            
            PDOmap param(mapentry.get_cached());
            boost::shared_ptr<Buffer> b = boost::make_shared<Buffer>(param.length/8);
            if(param.index < 0x1000){
                // TODO: check DummyUsage
            }else{
                ObjectStorage::ReadDelegate rd;
                ObjectStorage::WriteDelegate wd;
                if(read) rd = ObjectStorage::ReadDelegate(b.get(), &Buffer::read);
                if(read || write) wd = ObjectStorage::WriteDelegate(b.get(), &Buffer::write); // set writer for buffer setup or as write delegate
                size_t l = storage->map(param.index, param.sub_index, rd, wd);
                assert(l  == param.length/8);
            }
            
            frame.dlc += b->size;
            assert( frame.dlc <= 8 );
            b->clean();
            buffers.push_back(b);
        }
    }
    if(com_changed){
        uint8_t subs = dict(com_index, SUB_COM_NUM).value().get<uint8_t>();
        for(uint8_t i = SUB_COM_NUM+1; i <= subs; ++i){
            if(i == SUB_COM_COB_ID || i == SUB_COM_RESERVED) continue;
            try{
                storage->init(ObjectDict::Key(com_index, i));
            }
            catch (const std::out_of_range &){
                // entry was not provided, so skip it
            }
        }
    }
    if(map_changed){
        num_entry.set(map_num);
    }
    if((com_changed || map_changed) && cob_id.desc().writable){
        storage->init(ObjectDict::Key(com_index, SUB_COM_COB_ID));
        
        cob_id.set(NodeIdOffset<uint32_t>::apply(dict(com_index, SUB_COM_COB_ID).value(), storage->node_id_));
    }
        
    
}
PDOMapper::PDOMapper(const boost::shared_ptr<can::CommInterface> interface)
:interface_(interface)
{
}
bool PDOMapper::init(const boost::shared_ptr<ObjectStorage> storage, LayerStatus &status){
    boost::mutex::scoped_lock lock(mutex_);

    try{
        rpdos_.clear();

        const canopen::ObjectDict & dict = *storage->dict_;
        for(uint16_t i=0; i < 512 && rpdos_.size() < dict.device_info.nr_of_tx_pdo;++i){ // TPDOs of device
            if(!dict.has(TPDO_COM_BASE + i,0) && !dict.has(TPDO_MAP_BASE + i,0)) continue;

            boost::shared_ptr<RPDO> rpdo = RPDO::create(interface_,storage, TPDO_COM_BASE + i, TPDO_MAP_BASE + i);
            if(rpdo){
                rpdos_.insert(rpdo);
            }
        }
        // LOG("RPDOs: " << rpdos_.size());

        tpdos_.clear();
        for(uint16_t i=0; i < 512 && tpdos_.size() <  dict.device_info.nr_of_rx_pdo;++i){ // RPDOs of device
            if(!dict.has(RPDO_COM_BASE + i,0) && !dict.has(RPDO_MAP_BASE + i,0)) continue;

            boost::shared_ptr<TPDO> tpdo = TPDO::create(interface_,storage, RPDO_COM_BASE + i, RPDO_MAP_BASE + i);
            if(tpdo){
                tpdos_.insert(tpdo);
            }
        }
        // LOG("TPDOs: " << tpdos_.size());

        return true;
    }
    catch(const std::out_of_range &e){
        status.error(std::string("PDO error: ") + e.what());
        return false;
    }
}


bool PDOMapper::RPDO::init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
    boost::mutex::scoped_lock lock(mutex);
    listener_.reset();
    const canopen::ObjectDict & dict = *storage->dict_;
    parse_and_set_mapping(storage, com_index, map_index, true, false);
    
    PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(com_index, SUB_COM_COB_ID).value(), storage->node_id_) );

    if(buffers.empty() || pdoid.isInvalid()){
       return false;
    }

    frame = pdoid.header(true);

    transmission_type = dict(com_index, SUB_COM_TRANSMISSION_TYPE).value().get<uint8_t>();
    
    listener_ = interface_->createMsgListener(pdoid.header() ,can::CommInterface::FrameDelegate(this, &RPDO::handleFrame));
    
    return true;
}

bool PDOMapper::TPDO::init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
    boost::mutex::scoped_lock lock(mutex);
    const canopen::ObjectDict & dict = *storage->dict_;

    
    PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(com_index, SUB_COM_COB_ID).value(), storage->node_id_) );
    frame = pdoid.header();
    
    parse_and_set_mapping(storage, com_index, map_index, false, true);
    if(buffers.empty() || pdoid.isInvalid()){
       return false;
    }
    ObjectStorage::Entry<uint8_t> tt;
    storage->entry(tt, com_index, SUB_COM_TRANSMISSION_TYPE);
    transmission_type = tt.desc().value().get<uint8_t>();

    if(transmission_type != 1 && transmission_type <=240){
        tt.set(1); // enforce 1 for compatibility
    }
    return true;
}

void PDOMapper::TPDO::sync(){
    boost::mutex::scoped_lock lock(mutex);
    
    bool updated = false;
    size_t len = frame.dlc;
    uint8_t * dest = frame.data.c_array();
    for(std::vector< boost::shared_ptr<Buffer> >::iterator b_it = buffers.begin(); b_it != buffers.end(); ++b_it){
        Buffer &b = **b_it;
        if(len >= b.size){
            updated = b.read(dest, len) || updated;
            len -= b.size;
            dest += b.size;
        }else{
            // ERROR
        }
    }
    
    if( len != 0){
        // ERROR
    }
    if(updated){
        interface_->send( frame );
    }else{
        // TODO: Notify 
    }
}

void PDOMapper::RPDO::sync(LayerStatus &status){
    boost::mutex::scoped_lock lock(mutex);
    if((transmission_type >= 1 && transmission_type <= 240) || transmission_type == 0xFC){ // cyclic
        if(timeout > 0){
            --timeout;
        }else if(timeout == 0) {
            status.warn("RPDO timeout");
        }
    }
    if(transmission_type == 0xFC || transmission_type == 0xFD){
        if(frame.is_rtr){
            interface_->send(frame);
        }
    }
}

void PDOMapper::RPDO::handleFrame(const can::Frame & msg){
    size_t offset = 0;
    const uint8_t * src = msg.data.data();
    for(std::vector<boost::shared_ptr<Buffer> >::iterator it = buffers.begin(); it != buffers.end(); ++it){
        Buffer &b = **it;
        
        if( offset + b.size <= msg.dlc ){
            b.write(src+offset, b.size);
            offset += b.size;
        }else{
            // ERROR
        }
    }
    if( offset != msg.dlc ){
        // ERROR
    }
    {
        boost::mutex::scoped_lock lock(mutex);
        if(transmission_type >= 1 && transmission_type <= 240){
            timeout = transmission_type + 2;
        }else if(transmission_type == 0xFC || transmission_type == 0xFD){
            if(frame.is_rtr){
                timeout = 1+2;
            }
        }
    }
}

void PDOMapper::read(LayerStatus &status){
    boost::mutex::scoped_lock lock(mutex_);
    for(boost::unordered_set<boost::shared_ptr<RPDO> >::iterator it = rpdos_.begin(); it != rpdos_.end(); ++it){
        (*it)->sync(status);
    }
}
bool PDOMapper::write(){
    boost::mutex::scoped_lock lock(mutex_);
    for(boost::unordered_set<boost::shared_ptr<TPDO> >::iterator it = tpdos_.begin(); it != tpdos_.end(); ++it){
        (*it)->sync();
    }
    return true; // TODO: check for errors
}

bool PDOMapper::Buffer::read(uint8_t* b, const size_t len){
    boost::mutex::scoped_lock lock(mutex);
    if(size > len){
        BOOST_THROW_EXCEPTION( std::bad_cast() );
    }
    if(empty) return false;
    
    memcpy(b,&buffer[0], size);
    bool was_dirty = dirty;
    dirty = false;
    return was_dirty;
}
void PDOMapper::Buffer::write(const uint8_t* b, const size_t len){
    boost::mutex::scoped_lock lock(mutex);
    if(size > len){
        BOOST_THROW_EXCEPTION( std::bad_cast() );
    }
    empty = false;
    dirty = true;
    memcpy(&buffer[0], b, size);
}
void PDOMapper::Buffer::read(const canopen::ObjectDict::Entry &entry, String &data){
    boost::mutex::scoped_lock lock(mutex);
    time_point abs_time = get_abs_time(boost::chrono::seconds(1));
    if(size != data.size()){
        THROW_WITH_KEY(std::bad_cast(), ObjectDict::Key(entry));
    }
    if(empty){
        THROW_WITH_KEY(TimeoutException("PDO data empty"), ObjectDict::Key(entry));
    }
    if(dirty){
        data.assign(buffer.begin(), buffer.end());
        dirty = false;
    }
}
void PDOMapper::Buffer::write(const canopen::ObjectDict::Entry &entry, const String &data){
    boost::mutex::scoped_lock lock(mutex);
    if(size != data.size()){
        THROW_WITH_KEY(std::bad_cast(), ObjectDict::Key(entry));
    }
    empty = false;
    dirty = true;
    buffer.assign(data.begin(),data.end());
}
