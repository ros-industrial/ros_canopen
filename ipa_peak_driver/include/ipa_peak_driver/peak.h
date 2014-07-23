#ifndef H_IPA_PEAK_DRIVER
#define H_IPA_PEAK_DRIVER

#include <ipa_can_interface/asio_base.h>
#include <libpcan.h>

namespace ipa_can {

template<typename FrameDelegate, typename StateDelegate> class PeakDriver : public AsioDriver<FrameDelegate,StateDelegate,boost::asio::posix::stream_descriptor> {
    typedef AsioDriver<FrameDelegate,StateDelegate,boost::asio::posix::stream_descriptor> BaseClass;
public:    
    PeakDriver(FrameDelegate frame_delegate, StateDelegate state_delegate)
    : BaseClass(frame_delegate, state_delegate),timer_(BaseClass::io_service_),handle_(0)
    {}
    bool init(const std::string &device, unsigned int bitrate){
        State s = BaseClass::getState();
        if(s.driver_state == State::closed){
            boost::mutex::scoped_lock lock(handle_mutex_);
            
            device_ = device;
            bitrate_ = bitrate;
            
            handle_ = LINUX_CAN_Open(device.c_str(), O_RDWR);
            if(!handle_){
                BaseClass::setErrorCode(boost::system::error_code(nGetLastError(),boost::system::system_category()));
                return false;
            }
            
            DWORD ret = CAN_ERR_OK;
            
            CAN_Status(handle_); // reset all errors;

            WORD btr0btr1 = LINUX_CAN_BTR0BTR1(handle_, bitrate);
            ret = CAN_Init(handle_, btr0btr1, CAN_INIT_TYPE_ST);

            if(ret != CAN_ERR_OK){
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                CAN_Close(handle_);
                handle_ = 0;
                return false;
            }
            
            ret = CAN_ResetFilter(handle_);

            if(ret != CAN_ERR_OK)
            {
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                CAN_Close(handle_);
                handle_ = 0;
                return false;
            }
            
            boost::system::error_code ec;
            BaseClass::socket_.assign(LINUX_CAN_FileHandle(handle_),ec);
            
            BaseClass::setErrorCode(ec);
            
            if(ec){
                CAN_Close(handle_);
                handle_ = 0;
                return false;
            }
            timer_.expires_from_now(boost::posix_time::seconds(1));
            timer_.async_wait(boost::bind(&PeakDriver::checkStatus, this, boost::asio::placeholders::error));        
            BaseClass::setDriverState(State::open);
            return true;
        }
        // already initialized
        return false;
    }
    bool recover(){
        State s = BaseClass::getState();
        if(s.driver_state == State::open){
            shutdown();
            return init(device_, bitrate_);
        }
    }

    void shutdown(){
        {
            boost::mutex::scoped_lock lock(handle_mutex_);
            CAN_Close(handle_);
            handle_ = 0;
        }
        BaseClass::shutdown();
    }

    bool translateError(unsigned int internal_error, std::string & str){
        return false; // TODO
    }
    
protected:
    std::string device_;
    unsigned int bitrate_;
    
    void triggerReadSome(){
        if(handle_){
            BaseClass::socket_.async_read_some(boost::asio::null_buffers(), boost::bind( &PeakDriver::readFrame,this, boost::asio::placeholders::error));
        }
    }
    bool enqueue(const Frame & msg){
        boost::mutex::scoped_lock lock(handle_mutex_);

        TPCANMsg cmsg;
        cmsg.ID = msg.id;
        cmsg.MSGTYPE = (msg.is_extended?MSGTYPE_EXTENDED:0) | (msg.is_rtr?MSGTYPE_RTR:0);
        cmsg.LEN = msg.dlc < 8 ? msg.dlc : 8;
        for(int i=0; i < cmsg.LEN;++i)
            cmsg.DATA[i] = msg.data[i];
        
        DWORD ret = CAN_Write(handle_, &cmsg);
        
        if(ret && ret != CAN_ERR_QRCVEMPTY){
                BaseClass::setDriverState(State::open);
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                return false;
        }
        return true;
    }
    
    void checkStatus(const boost::system::error_code& error){
        if (error) return;
        if(handle_){
            boost::mutex::scoped_lock lock(handle_mutex_);
            DWORD err = CAN_Status(handle_);
            
            if(err && err != CAN_ERR_QRCVEMPTY){
                BaseClass::setDriverState(State::open);
                BaseClass::setInternalError(err);
                BaseClass::socket_.cancel();
            }else{
                timer_.expires_from_now(boost::posix_time::seconds(1));
                timer_.async_wait(boost::bind(&PeakDriver::checkStatus, this, boost::asio::placeholders::error));        
            }
            
        }else return;
        enqueue(Frame(1));
    }
    void readFrame(const boost::system::error_code& error){
        timer_.cancel();
        if(!error){
            boost::mutex::scoped_lock lock(handle_mutex_);

            TPCANRdMsg msg;
            DWORD ret = LINUX_CAN_Read(handle_,&msg);
            if(ret){
                BaseClass::setDriverState(State::open);
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                
                // TODO: switch to open?
                return;
            }
            
            if( msg.Msg.MSGTYPE & MSGTYPE_STATUS){
                BaseClass::setDriverState(State::open);
                BaseClass::setInternalError(msg.Msg.DATA[3]);
                // TODO: switch to open?
                
                return;
            }
            
            BaseClass::input_.id = msg.Msg.ID;
            BaseClass::input_.is_error = msg.Msg.MSGTYPE & MSGTYPE_STATUS;
            BaseClass::input_.is_extended = msg.Msg.MSGTYPE & MSGTYPE_EXTENDED;
            BaseClass::input_.is_rtr = msg.Msg.MSGTYPE & MSGTYPE_RTR;
            BaseClass::input_.dlc = msg.Msg.LEN;
            for(int i=0;i<msg.Msg.LEN && i < 8; ++i){
                BaseClass::input_.data[i] = msg.Msg.DATA[i];
            }
        }
        BaseClass::frameReceived(error);
    }
    
private:
    boost::asio::deadline_timer timer_;
    HANDLE handle_;
    boost::mutex handle_mutex_;
};
    
}; // namespace ipa_can
#endif
