#include <ipa_canopen_master/canopen.h>

using namespace ipa_canopen;

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

struct PDOid{
    uint32_t id:29;
    uint32_t extended:1;
    uint32_t no_rtr:1;
    uint32_t invalid:1;
    PDOid(uint32_t val){
        *(uint32_t*) this = val;
    }
    ipa_can::Header header() {
        return ipa_can::Header(id, extended);
    }
    const uint32_t get() const { return *(uint32_t*) this; }
};

struct PDOmap{
    uint8_t length;
    uint8_t sub_index;
    uint16_t index;
    PDOmap(uint32_t val){
        *(uint32_t*) this = val;
    }
};

#pragma pack(pop) /* pop previous alignment from stack */


const uint8_t SUB_COM_COB_ID = 1;
const uint8_t SUB_COM_TRANSMISSION_TYPE = 2;

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
void PDOMapper::PDO::parse_and_set_mapping(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index,
                                           const ObjectStorage::ReadDelegate &rd, const ObjectStorage::WriteDelegate &wd){
                            
    const ipa_canopen::ObjectDict & dict = *storage->dict_;
    
    ObjectStorage::Entry<uint8_t> num_entry;
    storage->entry(num_entry, map_index, SUB_MAP_NUM);

    uint8_t map_num = num_entry.desc().value().get<uint8_t>();
    
    bool map_changed = check_map_changed(map_num, dict, map_index);
    
    // disable PDO if needed
    if(map_changed || check_com_changed(dict, map_index)){
        ObjectStorage::Entry<uint32_t> cob_id;
        storage->entry(cob_id, com_index, SUB_COM_COB_ID);
        
        PDOid cur(cob_id.get(true));
        cur.invalid = 1;
        cob_id.set(cur.get());
    }

    
    if(map_num > 0 && map_num <= 0x40){ // actual mapping 
        if(map_changed){
            num_entry.set(0);
        }
        
        size_t offset = 0;
        for(uint8_t sub = 1; sub <=map_num; ++sub){
            ObjectStorage::Entry<uint32_t> mapentry;
            storage->entry(mapentry, map_index, sub);
            const HoldAny init = dict(map_index ,sub).init_val;
            if(!init.is_empty()) mapentry.set(init.get<uint32_t>());
            
            PDOmap param(mapentry.get(true));
            boost::shared_ptr<ObjectStorage::Buffer> b;
            if(param.index < 0x1000){
                b = ObjectStorage::Buffer::dummy(param.length/8);
            }else{
                b = storage->map(param.index, param.sub_index, rd, wd);
                assert(b->size() == param.length/8);
            }
            
            offset += b->size();
            assert( offset <= 8 );
            buffers.push_back(b);
        }
    }
    if(map_changed){
        num_entry.set(map_num);
    }
}
PDOMapper::PDOMapper(const boost::shared_ptr<ipa_can::Interface> interface)
:interface_(interface)
{
}
void PDOMapper::init(const boost::shared_ptr<ObjectStorage> storage){
    rpdos_.clear();
    
    const ipa_canopen::ObjectDict & dict = *storage->dict_;
    for(uint8_t i=0; i < dict.device_info.nr_of_tx_pdo;++i){ // TPDOs of device
        boost::shared_ptr<RPDO> rpdo = boost::make_shared<RPDO>(interface_);
        if(rpdo->init(storage, TPDO_COM_BASE + i, TPDO_MAP_BASE + i)){
            rpdos_.insert(rpdo);
        }
    }
    std::cout << "RPDOs: " << rpdos_.size() << std::endl;
    
    tpdos_.clear();
    for(uint8_t i=0; i < dict.device_info.nr_of_rx_pdo;++i){ // RPDOs of device
        boost::shared_ptr<TPDO> tpdo = boost::make_shared<TPDO>(interface_);
        if(tpdo->init(storage, RPDO_COM_BASE + i, RPDO_MAP_BASE + i)){
            tpdos_.insert(tpdo);
        }
    }
    std::cout << "TPDOs: " << tpdos_.size() << std::endl;
}


bool PDOMapper::RPDO::init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
    const ipa_canopen::ObjectDict & dict = *storage->dict_;
    parse_and_set_mapping(storage, com_index, map_index, ObjectStorage::ReadDelegate(this,&RPDO::read), ObjectStorage::WriteDelegate());
    
    PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(com_index, SUB_COM_COB_ID).value(), storage->node_id_) );

    if(buffers.empty() || pdoid.invalid){
       return false;     
    }
        
    frame = pdoid.header();
    frame.is_rtr = pdoid.no_rtr?0:1;
    
    transmission_type = dict(com_index, SUB_COM_TRANSMISSION_TYPE).value().get<uint8_t>();
    
    listener_ = interface_->createMsgListener(pdoid.header() ,ipa_can::Interface::FrameDelegate(this, &RPDO::handleFrame));

    return true;
}

bool PDOMapper::TPDO::init(const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index){
    const ipa_canopen::ObjectDict & dict = *storage->dict_;

    parse_and_set_mapping(storage, com_index, map_index, ObjectStorage::ReadDelegate(), ObjectStorage::WriteDelegate(this, &TPDO::write));
    
    PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(com_index, SUB_COM_COB_ID).value(), storage->node_id_) );
    frame = pdoid.header();
    
    if(buffers.empty() || pdoid.invalid){
       return false;     
    }
    
    ObjectStorage::Entry<uint8_t> tt;
    storage->entry(tt, com_index, SUB_COM_TRANSMISSION_TYPE);
    transmission_type = tt.desc().value().get<uint8_t>();
    
    if(transmission_type > 1 && transmission_type <=240){
        tt.set(1);
    }
    return true;
}

void PDOMapper::TPDO::sync(const uint8_t &counter){
    boost::mutex::scoped_lock lock(mutex);
    
    if(counter > 0 && (transmission_type >= 1 || transmission_type <= 240)){
            if((counter % transmission_type) != 0) return;
    }else if(!ready) return;
    
    size_t len = frame.dlc;
    uint8_t * dest = frame.data.c_array();
    for(std::vector< boost::shared_ptr<ObjectStorage::Buffer> >::iterator b_it = buffers.begin(); b_it != buffers.end(); ++b_it){
        ObjectStorage::Buffer &b = **b_it;
        size_t s = b.size();
        if(len >= s){
            b.read(dest, len);
            len -= s;
            dest += s;
        }else{
            // ERROR
        }
    }
    
    if( len != 0){
        // ERROR
    }
    interface_->send( frame );
    
    ready = false;
}

void PDOMapper::RPDO::sync(const uint8_t &counter){
    boost::mutex::scoped_lock lock(mutex);
    
    if(transmission_type >= 1 && transmission_type <=240){ 
        if(timeout > 0){
            --timeout;
        }else if(timeout == 0){
            throw TimeoutException();
        }
        
    }else if(transmission_type == 0xfc || transmission_type == 0xfd){
        timeout = 0;
    }
}

void PDOMapper::RPDO::read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data){
    boost::mutex::scoped_lock lock(mutex);
    
    if(timeout == 0){
        if(transmission_type >= 1 && transmission_type <=240){ 
            throw TimeoutException();
        }else if(transmission_type != 0xfc && transmission_type != 0xfd){
            return;
        }
    }else if (timeout > 0) return;
    
    if(frame.is_rtr){
        int test_timeout = timeout;
        interface_->send(frame);
        boost::system_time abs_time = boost::get_system_time() + boost::posix_time::seconds(1);
        while(timeout == test_timeout)
        {
            if(!cond.timed_wait(lock,abs_time))
            {
                throw TimeoutException();
            }
        }
    }else if(timeout == 0) {
        throw TimeoutException();
    }
}

void PDOMapper::RPDO::handleFrame(const ipa_can::Frame & msg){
    boost::mutex::scoped_lock lock(mutex);

    size_t offset = 0;
    const uint8_t * src = msg.data.data();
    for(std::vector<boost::shared_ptr<ObjectStorage::Buffer> >::iterator it = buffers.begin(); it != buffers.end(); ++it){
        ObjectStorage::Buffer &b = **it;
        // std::cout << "RPDO " << std::hex << b.entry->index << std::dec << " " << b.entry->sub_index << std::endl;
        
        size_t s = b.size();
        if( offset + s <= msg.dlc ){
            b.write(src+offset, s);
            offset += s;
        }else{
            // ERROR
        }
    }
    if( offset != msg.dlc ){
        // ERROR
    }
    
    if(transmission_type >= 1 && transmission_type <=240){ 
        timeout = transmission_type*2;
    }else{
        timeout = 1;
    }
    cond.notify_one();
}

void PDOMapper::sync(const uint8_t &counter){
    for(boost::unordered_set<boost::shared_ptr<RPDO> >::iterator it = rpdos_.begin(); it != rpdos_.end(); ++it){
        (*it)->sync(counter);
    }
    for(boost::unordered_set<boost::shared_ptr<TPDO> >::iterator it = tpdos_.begin(); it != tpdos_.end(); ++it){
        (*it)->sync(counter);
    }
}

bool PDOMapper::Buffer::read(uint8_t* b, const size_t len){
    boost::mutex::scoped_lock lock(mutex);
    if(size > len){
        throw std::bad_cast();
    }
    if(empty) return false;
    
    memcpy(b,&buffer[0], size);
    bool was_dirty = dirty;
    dirty = false;
    return !was_dirty;
}
void PDOMapper::Buffer::write(const uint8_t* b, const size_t len){
    boost::mutex::scoped_lock lock(mutex);
    if(size > len){
        throw std::bad_cast();
    }
    empty = false;
    dirty = true;
    memcpy(&buffer[0], b, size);
    lock.unlock();
    cond.notify_all();
}
void PDOMapper::Buffer::read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data){
    boost::mutex::scoped_lock lock(mutex);
    boost::system_time abs_time = boost::get_system_time() + boost::posix_time::seconds(1);
    if(size != data.size()){
        throw std::bad_cast();
    }
    while(empty){
        if(!cond.timed_wait(lock,abs_time))
        {
            throw TimeoutException();
        }
    }
    if(dirty){
        data.assign(buffer.begin(), buffer.end());
        dirty = false;
    }
}
void PDOMapper::Buffer::write(const ipa_canopen::ObjectDict::Entry &, const std::string &data){
    boost::mutex::scoped_lock lock(mutex);
    if(size != data.size()){
        throw std::bad_cast();
    }
    empty = false;
    dirty = true;
    buffer.assign(data.begin(),data.end());
}
