#pragma once

#include <sys/time.h>

#include <string>
#include <sstream>
#include <stdexcept>


namespace ephemerand {

// Based on https://www.agwa.name/projects/templates/

inline void build_string(std::ostream&) { }

template<class First, class... Rest>
inline void build_string(std::ostream& o, const First& value, const Rest&... rest) {
    o << value;
    build_string(o, rest...);
}

template<class... T>
std::string concat_string(const T&... value) {
    std::ostringstream o;
    build_string(o, value...);
    return o.str();
}

template<class... T>
std::runtime_error error(const T&... value) {
    std::ostringstream o;
    build_string(o, value...);
    return std::runtime_error(o.str());
}



namespace util {

inline uint64_t curr_time_us() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

std::string to_hex(std::string_view input, bool prefixed = false);
std::string from_hex(std::string_view input);

}

}


using ephemerand::util::to_hex;
using ephemerand::util::from_hex;
