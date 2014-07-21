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

void RPDOHandler::handleFrame(const ipa_can::Frame & msg){
    size_t offset = 0;
    const uint8_t * src = msg.data.data();
    for(std::vector<boost::shared_ptr<ObjectStorage::Buffer> >::iterator it = buffers_.begin(); it != buffers_.end(); ++it){
        ObjectStorage::Buffer &b = **it;
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
}

RPDOHandler::RPDOHandler(const boost::shared_ptr<ipa_can::Interface> interface, const ipa_can::Header &h, const std::vector<boost::shared_ptr<ObjectStorage::Buffer> > &buffers)
: listener_(interface->createMsgListener(h,ipa_can::Interface::FrameDelegate(this, &RPDOHandler::handleFrame))), buffers_(buffers)
{
    // test consistency of buffers
    size_t offset = 0;
    for(std::vector<boost::shared_ptr<ObjectStorage::Buffer> >::iterator it = buffers_.begin(); it != buffers_.end(); ++it){
        offset += (*it)->size();
    }
    assert( offset <= 8 );
    
}

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
void parse_and_set_mapping(std::vector<boost::shared_ptr<ObjectStorage::Buffer> > &buffers,
                           const boost::shared_ptr<ObjectStorage> &storage, const uint16_t &com_index, const uint16_t &map_index,
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
    setup_rpdos(storage);
    std::cout << "RPDOs: " << rpdos_.size() << std::endl;
    tpdos_.clear();
//     setup_tpdos(storage);
//     std::cout << "TPDOs: " << tpdos_.size() << std::endl;
    
}
void PDOMapper::setup_rpdos(const boost::shared_ptr<ObjectStorage> storage){

    const ObjectStorage::ReadDelegate rd(this, &PDOMapper::read);
    const ObjectStorage::WriteDelegate wd(this, &PDOMapper::write);
    
    const ipa_canopen::ObjectDict & dict = *storage->dict_;
    for(uint8_t i=0; i < dict.device_info.nr_of_tx_pdo;++i){ // TPDOs of device
        std::cout << "RPDO" << (int)i << std::endl;

        std::vector<boost::shared_ptr<ObjectStorage::Buffer> > buffers;
        
        parse_and_set_mapping(buffers, storage, TPDO_COM_BASE + i, TPDO_MAP_BASE + i, rd, wd);

        PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(TPDO_COM_BASE + i, SUB_COM_COB_ID).value(), storage->node_id_) );
        
        if(!buffers.empty()){
            RPDO rpdo;
            rpdo.transmission_type = dict(TPDO_COM_BASE + i, SUB_COM_TRANSMISSION_TYPE).value().get<uint8_t>();
            rpdo.rtr = pdoid.no_rtr ? 0 : 1;
            rpdo.handler = boost::make_shared<RPDOHandler>(interface_, pdoid.header(), buffers);
            rpdos_.insert(std::make_pair(pdoid.header(),rpdo));
        }
    }
}
void PDOMapper::setup_tpdos(const boost::shared_ptr<ObjectStorage> storage){

    const ObjectStorage::ReadDelegate rd(this, &PDOMapper::read);
    const ObjectStorage::WriteDelegate wd(this, &PDOMapper::write);
    
    const ipa_canopen::ObjectDict & dict = *storage->dict_;
    for(uint8_t i=0; i < dict.device_info.nr_of_tx_pdo;++i){ // TPDOs of device

        TPDO tpdo;
        
        parse_and_set_mapping(tpdo.buffers, storage, RPDO_COM_BASE + i, RPDO_MAP_BASE + i, rd, wd);
        
        if(!tpdo.buffers.empty()){
            PDOid pdoid( NodeIdOffset<uint32_t>::apply(dict(RPDO_COM_BASE + i, SUB_COM_COB_ID).value(), storage->node_id_) );
            tpdo.frame = pdoid.header();
            tpdos_.insert(std::make_pair(tpdo.frame, tpdo));
        }
    }
}

void PDOMapper::read(const ipa_canopen::ObjectDict::Entry &entry, std::string &data){
    // TODO: RTR request?
}
void PDOMapper::write(const ipa_canopen::ObjectDict::Entry &entry, const std::string &data){
    // TODO: event based transmission?
}
void PDOMapper::sync(){
    for(boost::unordered_map<ipa_can::Header, RPDO >::iterator r_it = rpdos_.begin(); r_it != rpdos_.end(); ++r_it){
        if(r_it->second.transmission_type == 0xfc || r_it->second.transmission_type == 0xfd){
            ipa_can::Frame f(r_it->first, 0);
            f.is_rtr = 1;
            interface_->send(f);
        }
    }
    for(boost::unordered_map<ipa_can::Header, TPDO >::iterator t_it = tpdos_.begin(); t_it != tpdos_.end(); ++t_it){
        TPDO & t = t_it->second;
        
        size_t len = t.frame.dlc;
        uint8_t * dest = t.frame.data.c_array();
        for(std::vector< boost::shared_ptr<ObjectStorage::Buffer> >:: iterator b_it = t.buffers.begin(); b_it != t.buffers.end(); ++b_it){
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
        interface_->send( t.frame );
        
    }
}