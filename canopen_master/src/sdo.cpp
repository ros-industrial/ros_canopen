#include <canopen_master/canopen.h>

using namespace canopen;

const uint8_t COMMAND_MASK =  (1<<7) | (1<<6) | (1<<5);
const uint8_t INITIATE_DOWNLOAD_REQUEST =  (0 << 5);
const uint8_t INITIATE_DOWNLOAD_RESPONSE =  (1 << 5);
const uint8_t DOWNLOAD_SEGMENT_REQUEST =  (1 << 5);
const uint8_t DOWNLOAD_SEGMENT_RESPONSE =  (3 << 5);
const uint8_t INITIATE_UPLOAD_REQUEST =  (2 << 5);
const uint8_t INITIATE_UPLOAD_RESPONSE =  (2 << 5);
const uint8_t UPLOAD_SEGMENT_REQUEST =  (3 << 5);
const uint8_t UPLOAD_SEGMENT_RESPONSE =  (0 << 5);
const uint8_t ABORT_TRANSFER_REQUEST =  (4 << 5);


#pragma pack(push) /* push current alignment to stack */
#pragma pack(1) /* set alignment to 1 byte boundary */

struct SDOid{
    uint32_t id:29;
    uint32_t extended:1;
    uint32_t dynamic:1;
    uint32_t invalid:1;
    SDOid(uint32_t val){
        *(uint32_t*) this = val;
    }
    can::Header header() {
        return can::Header(id, extended, false, false);
    }
};

struct InitiateShort{
    uint8_t :5;
    uint8_t command:3;
    uint16_t index;
    uint8_t sub_index;
    uint8_t reserved[4];
};

struct InitiateLong{
    uint8_t size_indicated:1;
    uint8_t expedited:1;
    uint8_t num:2;
    uint8_t :1;
    uint8_t command:3;
    uint16_t index;
    uint8_t sub_index;
    uint8_t payload[4];
    
    size_t data_size(){
        if(expedited && size_indicated) return 4-num;
        else if(!expedited && size_indicated) return payload[0] | (payload[3]<<8);
        else return 0;
    }
    size_t apply_buffer(const String &buffer){
        size_t size = buffer.size();
        size_indicated = 1;
        if(size > 4){
            expedited = 0;
            payload[0] = size & 0xFF;
            payload[3] = (size >> 8) & 0xFF;
            return 0;
        }else{
            expedited = 1;
            size_indicated = 1;
            num = 4-size;
            memcpy(payload, buffer.data(), size);
            return size;
        }
    }
};

struct SegmentShort{
    uint8_t :4;
    uint8_t toggle:1;
    uint8_t command:3;
    uint8_t reserved[7];
};

struct SegmentLong{
    uint8_t done:1;
    uint8_t num:3;
    uint8_t toggle:1;
    uint8_t command:3;
    uint8_t payload[7];
    size_t data_size(){
        return 7-num;
    }
    size_t apply_buffer(const String &buffer, const size_t offset){
        size_t size = buffer.size() - offset;
        if(size > 7) size = 7;
        else done = 1;
        num = 7 - size;
        memcpy(payload, buffer.data() + offset, size);
        return offset + size;
    }
};

struct DownloadInitiateRequest: public FrameOverlay<InitiateLong>{
    static const uint8_t command = 1;
    
    DownloadInitiateRequest(const Header &h, const canopen::ObjectDict::Entry &entry, const String &buffer, size_t &offset) : FrameOverlay(h) {
        data.command = command;
        data.index = entry.index;
        data.sub_index = entry.sub_index;
        offset = data.apply_buffer(buffer);
   }
    DownloadInitiateRequest(const can::Frame &f) : FrameOverlay(f){ }
};

struct DownloadInitiateResponse: public FrameOverlay<InitiateShort>{
    static const uint8_t command = 3;
    
    DownloadInitiateResponse(const can::Frame &f) : FrameOverlay(f){ }
    
    bool test(const can::Frame  &msg, uint32_t &reason){
        DownloadInitiateRequest req(msg);
        if(req.data.command ==  DownloadInitiateRequest::command && data.index == req.data.index && data.sub_index == req.data.sub_index){
            return true;
        }
        reason = 0x08000000; // General error
        return false;
    }
};

struct DownloadSegmentRequest: public FrameOverlay<SegmentLong>{
    static const uint8_t command = 0;
    
    DownloadSegmentRequest(const can::Frame &f) : FrameOverlay(f){ }
    
    DownloadSegmentRequest(const Header &h, bool toggle, const String &buffer, size_t& offset) : FrameOverlay(h) {
        data.command = command;
        data.toggle = toggle?1:0;
        offset = data.apply_buffer(buffer, offset);
    }
};

struct DownloadSegmentResponse : public FrameOverlay<SegmentShort>{
    static const uint8_t command = 1;
    DownloadSegmentResponse(const can::Frame &f) : FrameOverlay(f) {
    }
    bool test(const can::Frame  &msg, uint32_t &reason){
        DownloadSegmentRequest req(msg);
        if (req.data.command !=  DownloadSegmentRequest::command){
            reason = 0x08000000; // General error
            return false;
        }else if( data.toggle != req.data.toggle){
            reason = 0x05030000; // Toggle bit not alternated
            return false;
        }
        return true;
    }
};

struct UploadInitiateRequest: public FrameOverlay<InitiateShort>{
    static const uint8_t command = 2;
    UploadInitiateRequest(const Header &h, const canopen::ObjectDict::Entry &entry) : FrameOverlay(h) {
        data.command = command;
        data.index = entry.index;
        data.sub_index = entry.sub_index;
   }
    UploadInitiateRequest(const can::Frame &f) : FrameOverlay(f){ }
};

struct UploadInitiateResponse: public FrameOverlay<InitiateLong>{
    static const uint8_t command = 2;
    UploadInitiateResponse(const can::Frame &f) : FrameOverlay(f) { }
    bool test(const can::Frame  &msg, size_t size, uint32_t &reason){
        UploadInitiateRequest req(msg);
        if(req.data.command ==  UploadInitiateRequest::command && data.index == req.data.index && data.sub_index == req.data.sub_index){
                size_t ds = data.data_size();
                if(ds == 0  || size == 0 || ds >= size) { // should be ==, but >= is needed for Elmo, it responses with more byte than requested
                    if(!data.expedited || (ds <= 4 && size <= 4)) return true;
                }else{
                    reason = 0x06070010; // Data type does not match, length of service parameter does not match                    
                    return false;
                }
        }
        reason = 0x08000000; // General error
        return false;
    }
    bool read_data(String & buffer, size_t & offset, size_t & total){
        if(data.size_indicated && total == 0){
            total = data.data_size();
            buffer.resize(total);
        }
        if(data.expedited){
            memcpy(&buffer.front(), data.payload, buffer.size());
            offset = buffer.size();
            return true;
        }
        return false;
    }
};
struct UploadSegmentRequest: public FrameOverlay<SegmentShort>{
    static const uint8_t command = 3;
    UploadSegmentRequest(const Header &h, bool toggle) : FrameOverlay(h) {
        data.command = command;
        data.toggle = toggle?1:0;
   }
    UploadSegmentRequest(const can::Frame &f) : FrameOverlay(f) { }
};

struct UploadSegmentResponse : public FrameOverlay<SegmentLong>{
    static const uint8_t command = 0;
    UploadSegmentResponse(const can::Frame &f) : FrameOverlay(f) {
    }
    bool test(const can::Frame  &msg, uint32_t &reason){
        UploadSegmentRequest req(msg);
        if(req.data.command !=  UploadSegmentRequest::command){
            reason = 0x08000000; // General error
            return false;
        }else if( data.toggle != req.data.toggle){
            reason = 0x05030000; // Toggle bit not alternated
            return false;
        }
        return true;
    }
    bool read_data(String & buffer, size_t & offset, const size_t & total){
        uint32_t n = data.data_size();
        if(total == 0){
            buffer.resize(offset + n);
        }
        if(offset +  n <= buffer.size()){
            memcpy(&buffer[offset], data.payload, n);
            offset +=  n;
            return true;
        }
        return false;
    }
};

struct AbortData{
    uint8_t :5;
    uint8_t command:3;
    uint16_t index;
    uint8_t sub_index;
    uint32_t reason;
    
    const char * text(){
        switch(reason){
        case 0x05030000: return "Toggle bit not alternated.";
        case 0x05040000: return "SDO protocol timed out.";
        case 0x05040001: return "Client/server command specifier not valid or unknown.";
        case 0x05040002: return "Invalid block size (block mode only).";
        case 0x05040003: return "Invalid sequence number (block mode only).";
        case 0x05040004: return "CRC error (block mode only).";
        case 0x05040005: return "Out of memory.";
        case 0x06010000: return "Unsupported access to an object.";
        case 0x06010001: return "Attempt to read a write only object.";
        case 0x06010002: return "Attempt to write a read only object.";
        case 0x06020000: return "Object does not exist in the object dictionary.";
        case 0x06040041: return "Object cannot be mapped to the PDO.";
        case 0x06040042: return "The number and length of the objects to be mapped would exceed PDO length.";
        case 0x06040043: return "General parameter incompatibility reason.";
        case 0x06040047: return "General internal incompatibility in the device.";
        case 0x06060000: return "Access failed due to an hardware error.";
        case 0x06070010: return "Data type does not match, length of service parameter does not match";
        case 0x06070012: return "Data type does not match, length of service parameter too high";
        case 0x06070013: return "Data type does not match, length of service parameter too low";
        case 0x06090011: return "Sub-index does not exist.";
        case 0x06090030: return "Invalid value for parameter (download only).";
        case 0x06090031: return "Value of parameter written too high (download only).";
        case 0x06090032: return "Value of parameter written too low (download only).";
        case 0x06090036: return "Maximum value is less than minimum value.";
        case 0x060A0023: return "Resource not available: SDO connection";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data cannot be transferred or stored to the application.";
        case 0x08000021: return "Data cannot be transferred or stored to the application because of local control.";
        case 0x08000022: return "Data cannot be transferred or stored to the application because of the present device state.";
        case 0x08000023: return "Object dictionary dynamic generation fails or no object dictionary is present (e.g.object dictionary is generated from file and generation fails because of an file error).";
        case 0x08000024: return "No data available";
        default: return "Abort code is reserved";
        }
    }
};

struct AbortTranserRequest: public FrameOverlay<AbortData>{
    static const uint8_t command = 4;
    AbortTranserRequest(const can::Frame &f) : FrameOverlay(f) {}
    AbortTranserRequest(const Header &h, uint16_t index, uint8_t sub_index, uint32_t reason) : FrameOverlay(h) {
        data.command = command;
        data.index = index;
        data.sub_index = sub_index;
        data.reason = reason;
   }
};

#pragma pack(pop) /* pop previous alignment from stack */

void SDOClient::abort(uint32_t reason){
    if(current_entry){
        interface_->send(last_msg = AbortTranserRequest(client_id, current_entry->index, current_entry->sub_index, reason));
    }
}

bool SDOClient::processFrame(const can::Frame & msg){
    if(msg.dlc != 8) return false;
    
    uint32_t reason = 0;
    switch(msg.data[0] >> 5){
        case DownloadInitiateResponse::command:
        {
            DownloadInitiateResponse resp(msg);
            if(resp.test(last_msg, reason) ){
                if(offset < total){
                    interface_->send(last_msg = DownloadSegmentRequest(client_id, false, buffer, offset));
                }else{
                    done = true;
                }
            }
            break;
        }
        case DownloadSegmentResponse::command:
        {
            DownloadSegmentResponse resp(msg);
            if( resp.test(last_msg, reason) ){
                if(offset < total){
                    interface_->send(last_msg = DownloadSegmentRequest(client_id, !resp.data.toggle, buffer, offset));
                }else{
                    done = true;
                }
            }
            break;
        }
            
        case UploadInitiateResponse::command:
        {
            UploadInitiateResponse resp(msg);
            if( resp.test(last_msg, total, reason) ){
                if(resp.read_data(buffer, offset, total)){
                    done = true;
                }else{
                    interface_->send(last_msg = UploadSegmentRequest(client_id, false));
                }
            }
            break;
        }
        case UploadSegmentResponse::command:
        {
            UploadSegmentResponse resp(msg);
            if( resp.test(last_msg, reason) ){
                if(resp.read_data(buffer, offset, total)){
                    if(resp.data.done || offset == total){
                        done = true;
                    }else{
                        interface_->send(last_msg = UploadSegmentRequest(client_id, !resp.data.toggle));
                    }
                }else{
                    // abort, size mismatch
                    LOG("abort, size mismatch" << buffer.size() << " " << resp.data.data_size());
                    reason = 0x06070010; // Data type does not match, length of service parameter does not match
                }
            }
            break;
        }
        case AbortTranserRequest::command:
            LOG("abort" << std::hex << (uint32_t) AbortTranserRequest(msg).data.index << "#"<< std::dec << (uint32_t) AbortTranserRequest(msg).data.sub_index << ", reason: " << AbortTranserRequest(msg).data.text());
            offset = 0;
            return false;
            break;
    }
    if(reason){
        abort(reason);
        offset = 0;
        return false;
    }
    return true;

}    
    
void SDOClient::init(){
    assert(storage_);
    assert(interface_);
    const canopen::ObjectDict & dict = *storage_->dict_;

    try{
        client_id = SDOid(NodeIdOffset<uint32_t>::apply(dict(0x1200, 1).value(), storage_->node_id_)).header();
    }
    catch(...){
        client_id = can::MsgHeader(0x600+ storage_->node_id_);
    }
    
    last_msg = AbortTranserRequest(client_id, 0,0,0);
    current_entry = 0;

    can::Header server_id;
    try{
        server_id = SDOid(NodeIdOffset<uint32_t>::apply(dict(0x1200, 2).value(), storage_->node_id_)).header();
    }
    catch(...){
        server_id = can::MsgHeader(0x580+ storage_->node_id_);
    }
    reader_.listen(interface_, server_id);
}

void SDOClient::transmitAndWait(const canopen::ObjectDict::Entry &entry, const String &data,  String *result){
    buffer = data;
    offset = 0;
    total = buffer.size();
    current_entry = &entry;
    done = false;

    can::BufferedReader::ScopedEnabler enabler(reader_);

    if(result){
        interface_->send(last_msg = UploadInitiateRequest(client_id, entry));
    }else{
        interface_->send(last_msg = DownloadInitiateRequest(client_id, entry, buffer, offset));
    }

    boost::this_thread::disable_interruption di;
    can::Frame msg;

    while(!done){
        if(!reader_.read(&msg,boost::chrono::seconds(1)))
        {
            abort(0x05040000); // SDO protocol timed out.
            LOG("Did not receive a response message");
            break;
        }
        if(!processFrame(msg)){
            LOG("Could not process message");
            break;
        }
    }
    if(offset == 0 || offset != total){
        THROW_WITH_KEY(TimeoutException("SDO"), ObjectDict::Key(*current_entry));
    }

    if(result) *result=buffer;

}

void SDOClient::read(const canopen::ObjectDict::Entry &entry, String &data){
    boost::timed_mutex::scoped_lock lock(mutex, boost::chrono::seconds(2));
    if(lock){
        transmitAndWait(entry, data, &data);
    }else{
        THROW_WITH_KEY(TimeoutException("SDO read"), ObjectDict::Key(entry));
    }
}
void SDOClient::write(const canopen::ObjectDict::Entry &entry, const String &data){
    boost::timed_mutex::scoped_lock lock(mutex, boost::chrono::seconds(2));
    if(lock){
        transmitAndWait(entry, data, 0);
    }else{
        THROW_WITH_KEY(TimeoutException("SDO write"), ObjectDict::Key(entry));
    }
}
