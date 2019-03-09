#include <string>
#include <map>

#include <blake2.h>

#include <hoytech/protected_queue.h>

#include "ephemerand/ublox.h"
#include "ephemerand/gps_utils.h"
#include "ephemerand/util.h"


namespace ephemerand {




struct SatInfo {
    uint64_t svprn;
    uint64_t week;
    uint64_t time_of_week;
    std::string almanac;
};

using SatTable = std::map<uint64_t, SatInfo>;


bool check_sats(SatTable &sats) {
    if (sats.size() != 31) return false;

    uint64_t week = 0, time_of_week = 0;

    for (auto &[svprn, sat] : sats) {
        (void) svprn;

        if (week == 0) {
            week = sat.week;
            time_of_week = sat.time_of_week;
            continue;
        }

        if (week != sat.week || time_of_week != sat.time_of_week) return false;
    }

    return true;
}


std::string hash_almanac(SatTable &sats) {
    std::string fullAlmanac;

    for (auto &[svprn, sat] : sats) {
        (void) svprn;
        fullAlmanac += sat.almanac;
    }

    uint8_t hash[32];

    blake2b(hash, reinterpret_cast<uint8_t*>(fullAlmanac.data()), nullptr, sizeof(hash), fullAlmanac.size(), 0);

    return std::string(reinterpret_cast<char*>(hash), sizeof(hash));
}


void cmd_run(std::string device, bool verbose) {
    ephemerand::Ublox ub(device);

    ub.connect();
    ub.run();


    SatTable sats;


    while(1) {
        auto msg = ub.output_queue.pop();

        if (auto m = std::get_if<UbloxMessage_Version>(&msg)) {
            if (verbose) std::cout << "# Connection OK. SW: " << m->software_version << " HW: " << m->hardware_version << std::endl;
            ub.pollAlmanac();
            ub.pollSubframeBuffer();
        } else if (auto m = std::get_if<UbloxMessage_AlmanacMissing>(&msg)) {
            if (verbose) std::cout << "# Almanac missing: " << m->svprn << std::endl;
        } else if (auto m = std::get_if<UbloxMessage_AlmanacData>(&msg)) {
            char *buf = m->data.data();

            uint64_t toa = getbitu(buf, 48, 8) * 4096;
            time_t t = decode_gps_time(m->issue_week, toa);

            if (verbose) std::cout << "# Almanac data: " << m->svprn << " issue week: " << m->issue_week << " toa: " << toa << " data: " << to_hex(m->data) << " timestamp: " << t << std::endl;

            sats[m->svprn] = { m->svprn, m->issue_week, toa, m->data };

            if (check_sats(sats)) {
                std::cout << "rand " << to_hex(hash_almanac(sats)) << " " << m->issue_week << " " << toa << std::endl;
            }
        }
    }
}

}
