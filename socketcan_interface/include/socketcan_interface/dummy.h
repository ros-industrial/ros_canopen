#ifndef SOCKETCAN_INTERFACE_DUMMY_H
#define SOCKETCAN_INTERFACE_DUMMY_H

#include "interface.h"
#include "dispatcher.h"
#include "string.h"
#include <boost/algorithm/string.hpp>

namespace can{

class DummyInterface : public CommInterface{
    typedef FilteredDispatcher<const unsigned int, CommInterface::FrameListener> FrameDispatcher;
    typedef boost::unordered_multimap<std::string, Frame> Map;
    FrameDispatcher frame_dispatcher_;
    Map map_;
    const bool loopback_;

    bool add_noconv(const std::string &k, const Frame &v, bool multi){
        if(multi || map_.find(k) == map_.end()){
              map_.insert( std::make_pair(boost::to_lower_copy(k), v));
              return true;
        }
        return false;
    }
public:
    DummyInterface(bool loopback) : loopback_(loopback) {}

    bool add(const std::string &k, const Frame &v, bool multi){
        return add_noconv(boost::to_lower_copy(k), v, multi);
    }
    bool add(const Frame &k, const Frame &v, bool multi){
        return add_noconv(tostring(k,true), v, multi);
    }
    bool add(const std::string &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    bool add(const Frame &k, const std::string &v, bool multi){
        return add(k, toframe(v), multi);
    }
    virtual bool send(const Frame & msg){
        if(loopback_) frame_dispatcher_.dispatch(msg);
        try{
            std::pair <Map::iterator, Map::iterator> r = map_.equal_range(tostring(msg, true));
            for (Map::iterator it=r.first; it!=r.second; ++it){
                frame_dispatcher_.dispatch(it->second);
            }
        }
        catch(const std::out_of_range &e){
        }
        return true;
    }

    virtual FrameListener::Ptr createMsgListener(const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(delegate);
    }
    virtual FrameListener::Ptr createMsgListener(const Frame::Header&h , const FrameDelegate &delegate){
        return frame_dispatcher_.createListener(h, delegate);
    }

};


}

#endif
