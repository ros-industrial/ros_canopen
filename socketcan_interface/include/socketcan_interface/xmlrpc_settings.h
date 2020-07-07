#ifndef SOCKETCAN_INTERFACE_XMLRPC_SETTINGS_H
#define SOCKETCAN_INTERFACE_XMLRPC_SETTINGS_H
#include <socketcan_interface/logging.h>

#include <socketcan_interface/settings.h>
#include "xmlrpcpp/XmlRpcValue.h"
#include <sstream>
#include <string>

class XmlRpcSettings : public can::Settings {
public:
    XmlRpcSettings() {}
    XmlRpcSettings(const XmlRpc::XmlRpcValue &v) : value_(v) {}
    XmlRpcSettings& operator=(const XmlRpc::XmlRpcValue &v) { value_ = v; return *this; }
private:
    virtual bool getRepr(const std::string &name, std::string & repr) const {
        const XmlRpc::XmlRpcValue *value = &value_;

        std::string n = name;
        size_t delim_pos;
        while (value->getType() == XmlRpc::XmlRpcValue::TypeStruct && (delim_pos = n.find('/')) != std::string::npos){
            std::string segment = n.substr(0, delim_pos);
            if (!value->hasMember(segment)) return false;
            value = &((*value)[segment]);
            n.erase(0, delim_pos+1);
        }
        if(value->hasMember(n)){
            std::stringstream sstr;
            sstr << (*value)[n];
            repr = sstr.str();
            return true;
        }
        return false;
    }
    XmlRpc::XmlRpcValue value_;

};

#endif
