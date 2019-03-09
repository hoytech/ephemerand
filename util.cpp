#include <stdlib.h>

#include <string_view>
#include <string>

#include <ephemerand/util.h>


namespace ephemerand { namespace util {

std::string to_hex(std::string_view input, bool prefixed) {
    size_t prefix_offset = (prefixed ? 2 : 0);

    std::string output(2*input.length() + prefix_offset, '\0');

    if (prefixed) {
        output[0] = '0';
        output[1] = 'x';
    }

    static const char *lookup = "0123456789abcdef";

    for (size_t i = 0; i < input.length(); i++) {
        output[i*2 + prefix_offset] = lookup[(input[i] >> 4) & 0x0f];
        output[i*2 + 1 + prefix_offset] = lookup[input[i] & 0x0f];
    }

    return output;
}

std::string from_hex(std::string_view input) {
    if ((input.length() % 2) != 0) throw ephemerand::error("from_hex needs even length string");

    if (input.length() >= 2 && input.substr(0,2) == "0x") input = input.substr(2);

    std::string output(input.length()/2, '\0');

    auto decode = [](unsigned char c){
        if ('0' <= c && c <= '9') return c - '0';
        else if ('a' <= c && c <= 'f') return c - 'a' + 10;
        else if ('A' <= c && c <= 'F') return c - 'A' + 10;
        else throw ephemerand::error("unexpected character in from_hex: ", (int)c);
    };

    for(size_t i=0; i<input.length(); i+=2) {
        output[i/2] = (decode(input[i]) << 4) | decode(input[i+1]);
    }

    return output;
}

}}
