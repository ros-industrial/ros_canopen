/*
 * Copyright 2019 Fraunhofer
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef SOCKETCAN_INTERFACE_STRING_H
#define SOCKETCAN_INTERFACE_STRING_H

#include "interface.h"
#include "filter.h"
#include <sstream>

namespace can {

bool hex2dec(uint8_t& d, const char& h);

bool hex2buffer(std::string& out, const std::string& in_raw, bool pad);

bool dec2hex(char& h, const uint8_t& d, bool lc);

std::string byte2hex(const uint8_t& d, bool pad, bool lc);


std::string buffer2hex(const std::string& in, bool lc);

std::string tostring(const Header& h, bool lc);

Header toheader(const std::string& s);

std::string tostring(const Frame& f, bool lc);

Frame toframe(const std::string& s);

template<class T> FrameFilterSharedPtr tofilter(const T  &ct);
template<> FrameFilterSharedPtr tofilter(const std::string &s);
template<> FrameFilterSharedPtr tofilter(const uint32_t &id);

FrameFilterSharedPtr tofilter(const char* s);

template <typename T> FilteredFrameListener::FilterVector tofilters(const T& v) {
    FilteredFrameListener::FilterVector filters;
    for(size_t i = 0; i < static_cast<size_t>(v.size()); ++i){
        filters.push_back(tofilter(v[i]));
    }
    return filters;
}

}  // namespace can

std::ostream& operator <<(std::ostream& stream, const can::Header& h);
std::ostream& operator <<(std::ostream& stream, const can::Frame& f);

#endif  // SOCKETCAN_INTERFACE_STRING_H
