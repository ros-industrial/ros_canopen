#include <canopen_master/canopen.h>

using namespace canopen;

#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

struct EMCYid{
    uint32_t id:29;
    uint32_t extended:1;
    uint32_t :1;
    uint32_t invalid:1;
    EMCYid(uint32_t val){
        *(uint32_t*) this = val;
    }
    can::Header header() {
        return can::Header(id, extended, false, false);
    }
    const uint32_t get() const { return *(uint32_t*) this; }
};

struct EMCYfield{
    uint32_t error_code:16;
    uint32_t addition_info:16;
    EMCYfield(uint32_t val){
        *(uint32_t*) this = val;
    }
};

struct EMCYmsg{
    uint16_t error_code;
    uint8_t error_register;
    uint8_t manufacturer_specific_error_field[5];

    struct Frame: public FrameOverlay<EMCYmsg>{
        Frame(const can::Frame &f) : FrameOverlay(f){ }
    };
};

#pragma pack(pop) /* pop previous alignment from stack */

void EMCYHandler::handleEMCY(const can::Frame & msg){
    EMCYmsg::Frame em(msg);
    error_register_ = em.data.error_register;
}
const uint8_t EMCYHandler::error_register(){
    if(!emcy_listener_) error_register_ = error_register_obj_.get();
    return error_register_;
}

void EMCYHandler::init(LayerStatus &status){
    read(status);
}
void EMCYHandler::recover(){
    error_register_ = 0;
    num_errors_.set(0);
}

void EMCYHandler::read(LayerStatus &status){
    if(error_register() & (~32)){ // mask device specific error
        status.error("Node has emergency error");
    }
}
void EMCYHandler::diag(LayerReport &report){
    read(report);

    if(error_register_){
        report.add("error_register", (uint32_t) error_register_);

        uint8_t num = num_errors_.get();
        std::stringstream buf;
        for(size_t i= 0; i <num; ++i) {
            if( i!= 0){
                buf << ", ";
            }
            EMCYfield field(storage_->entry<uint32_t>(0x1003,i+1).get());
            buf << std::hex << field.error_code << "#" << field.addition_info;
        }
        report.add("errors", buf.str());

    }
}
EMCYHandler::EMCYHandler(const boost::shared_ptr<can::CommInterface> interface, const boost::shared_ptr<ObjectStorage> storage): storage_ (storage), error_register_(0){
    storage_->entry(error_register_obj_, 0x1001);

    storage_->entry(num_errors_, 0x1003,0);

    try{
        EMCYid emcy_id(storage_->entry<uint32_t>(0x1014).get_cached());
        emcy_listener_ = interface->createMsgListener( emcy_id.header(), can::CommInterface::FrameDelegate(this, &EMCYHandler::handleEMCY));
    }
    catch(...){
       // pass
    }
}
