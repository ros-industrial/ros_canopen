#ifndef SOCKETCAN_INTERFACE_STRING_H
#define SOCKETCAN_INTERFACE_STRING_H

#include "interface.h"
#include <sstream>

namespace can{

bool hex2dec(uint8_t& d, const char& h);

bool hex2buffer(std::string& out, const std::string& in_raw, bool pad);

bool dec2hex(char& h, const uint8_t& d, bool lc);

std::string byte2hex(const uint8_t& d, bool pad, bool lc);


std::string buffer2hex(const std::string& in, bool lc);

std::string tostring(const Header& h, bool lc);

Header toheader(const std::string& s);

std::string tostring(const Frame& f, bool lc);

Frame toframe(const std::string& s);

}

std::ostream& operator <<(std::ostream& stream, const can::Header& h);
std::ostream& operator <<(std::ostream& stream, const can::Frame& f);

#endif
