#ifndef SOCKETCAN_INTERFACE_SETTINGS_H
#define SOCKETCAN_INTERFACE_SETTINGS_H

#include <map>
#include <string>
#include <boost/lexical_cast.hpp>

namespace can {
  class Settings
  {
  public:
      template <typename T> T get_optional(const std::string &n, const T& def) const {
          std::string repr;
          if(!getRepr(n, repr)){
              return def;
          }
          return boost::lexical_cast<T>(repr);
      }
      template <typename T> bool get(const std::string &n, T& val) const {
          std::string repr;
          if(!getRepr(n, repr)) return false;
          val =  boost::lexical_cast<T>(repr);
          return true;
      }
      virtual ~Settings() {}
  private:
    virtual bool getRepr(const std::string &n, std::string & repr) const = 0;
  };

} // can

#endif
