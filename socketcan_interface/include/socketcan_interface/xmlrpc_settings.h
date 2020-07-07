#ifndef SOCKETCAN_INTERFACE_XMLRPC_SETTINGS_H
#define SOCKETCAN_INTERFACE_XMLRPC_SETTINGS_H

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
    virtual bool getRepr(const std::string &n, std::string & repr) const {
        if(value_.hasMember(n)){
            std::stringstream sstr;
            sstr << const_cast< XmlRpc::XmlRpcValue &>(value_)[n]; // does not write since already existing
            repr = sstr.str();
            return true;
        }
        return false;
    }
    XmlRpc::XmlRpcValue value_;

};

#endif
