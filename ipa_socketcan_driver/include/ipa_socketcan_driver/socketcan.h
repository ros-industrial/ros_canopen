#ifndef H_IPA_SOCKETCAN_DRIVER
#define H_IPA_SOCKETCAN_DRIVER

#include <ipa_can_interface/asio_base.h>
#include <boost/bind.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
 
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>

namespace ipa_can {

template<typename FrameDelegate, typename StateDelegate> class SocketCANDriver : public AsioDriver<FrameDelegate,StateDelegate,boost::asio::posix::stream_descriptor> {
    typedef AsioDriver<FrameDelegate,StateDelegate,boost::asio::posix::stream_descriptor> BaseClass;
    const bool loopback_;
public:    
    SocketCANDriver(FrameDelegate frame_delegate, StateDelegate state_delegate, bool loopback = false)
    : BaseClass(frame_delegate, state_delegate), loopback_(loopback)
    {}
    
    bool doesLoopBack() const{
        return loopback_;
    }

    bool init(const std::string &device, unsigned int bitrate){
        State s = BaseClass::getState();
        if(s.driver_state == State::closed){
            device_ = device;
            if(bitrate != 0) return false; // not supported, TODO: use libsocketcan

            int sc = socket( PF_CAN, SOCK_RAW, CAN_RAW );
            if(sc < 0){
                BaseClass::setErrorCode(boost::system::error_code(sc,boost::system::system_category()));
                return false;
            }
            
            struct ifreq ifr;
            strcpy(ifr.ifr_name, device_.c_str());
            int ret = ioctl(sc, SIOCGIFINDEX, &ifr);

            if(ret != 0){
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }

            can_err_mask_t err_mask = ( CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF ); //TODO select errors to track

            ret = setsockopt(sc, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
               &err_mask, sizeof(err_mask));
            
            if(ret != 0){
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }
            
            if(loopback_){
                int recv_own_msgs = 1; /* 0 = disabled (default), 1 = enabled */
                ret = setsockopt(sc, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
                
                if(ret != 0){
                    BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                    close(sc);
                    return false;
                }
            }
            
            struct sockaddr_can addr;
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            ret = bind( sc, (struct sockaddr*)&addr, sizeof(addr) );            

            if(ret != 0){
                BaseClass::setErrorCode(boost::system::error_code(ret,boost::system::system_category()));
                close(sc);
                return false;
            }
            
            boost::system::error_code ec;
            BaseClass::socket_.assign(sc,ec);
            
            BaseClass::setErrorCode(ec);
            
            if(ec){
                close(sc);
                return false;
            }
            BaseClass::setDriverState(State::open);
            return true;
        }
        return false;
    }
    bool recover(){
        State s = BaseClass::getState();
        if(s.driver_state == State::open){
            BaseClass::shutdown();
            return init(device_, 0);
        }
        return false;
    }
    bool translateError(unsigned int internal_error, std::string & str){
        return false; // TODO
    }
protected:
    std::string device_;
    can_frame frame_;
    
    void triggerReadSome(){
        boost::mutex::scoped_lock lock(send_mutex_);
        BaseClass::socket_.async_read_some(boost::asio::buffer(&frame_, sizeof(frame_)), boost::bind( &SocketCANDriver::readFrame,this, boost::asio::placeholders::error));
    }
    
    bool enqueue(const Frame & msg){
        boost::mutex::scoped_lock lock(send_mutex_); //TODO: timed try lock

        can_frame frame;
        frame.can_id = msg.id | (msg.is_extended?CAN_EFF_FLAG:0) | (msg.is_rtr?CAN_RTR_FLAG:0);;
        frame.can_dlc = msg.dlc;
        
        
        for(int i=0; i < frame.can_dlc;++i)
            frame.data[i] = msg.data[i];
        
        boost::system::error_code ec;
        boost::asio::write(BaseClass::socket_, boost::asio::buffer(&frame, sizeof(frame)),boost::asio::transfer_all(), ec);
        if(ec){
            LOG("FAILED " << ec);
            BaseClass::setErrorCode(ec);
            BaseClass::setDriverState(State::open);
            return false;
        }
        
        return true;
    }
    
    void readFrame(const boost::system::error_code& error){
        if(!error){
            if(frame_.can_id & CAN_ERR_FLAG){ // error message
                // TODO
                LOG("error");
            }

            BaseClass::input_.is_extended = frame_.can_id & CAN_EFF_FLAG;
            BaseClass::input_.id = frame_.can_id & (BaseClass::input_.is_extended ? CAN_EFF_MASK : CAN_SFF_MASK);
            BaseClass::input_.is_error = frame_.can_id & CAN_ERR_FLAG;
            BaseClass::input_.is_rtr = frame_.can_id & CAN_RTR_FLAG;
            BaseClass::input_.dlc = frame_.can_dlc;
            for(int i=0;i<frame_.can_dlc && i < 8; ++i){
                BaseClass::input_.data[i] = frame_.data[i];
            }
        }
        BaseClass::frameReceived(error);
    }
private:
    boost::mutex send_mutex_;
};
    
} // namespace ipa_can
#endif
