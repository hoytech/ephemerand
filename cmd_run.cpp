#include <string>
#include <map>

#include <blake2.h>

#include <hoytech/protected_queue.h>
#include <hoytech/timer.h>

#include "ephemerand/ublox.h"
#include "ephemerand/gps_utils.h"
#include "ephemerand/util.h"


namespace ephemerand {


static const uint64_t sats_needed = 31;


struct SatInfo {
    uint64_t svprn = 0;
    uint64_t week = 0;
    uint64_t time_of_week = 0;
    std::string almanac;
};

using SatTable = std::map<uint64_t, SatInfo>;


bool check_sats(SatTable &sats) {
    if (sats.size() != sats_needed) return false;

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


void cmd_run(std::string device, bool verbose, bool loop) {
    ephemerand::Ublox ub(device);

    ub.connect();
    ub.run();


    SatTable sats;
    std::string curr_rand;


    hoytech::timer pollAlmanacTimer;

    pollAlmanacTimer.repeat(5000000, [&]{
        ub.pollAlmanac();
    });


    while(1) {
        auto msg = ub.output_queue.pop();

        if (auto m = std::get_if<UbloxMessage_Version>(&msg)) {
            if (verbose) std::cout << "# Connection OK. SW: " << m->software_version << " HW: " << m->hardware_version << std::endl;
            ub.pollAlmanac();
            pollAlmanacTimer.run();
        } else if (std::get_if<UbloxMessage_AlmanacMissing>(&msg)) {
            // Ignore
        } else if (auto m = std::get_if<UbloxMessage_AlmanacData>(&msg)) {
            char *buf = m->data.data();

            uint64_t toa = getbitu(buf, 48, 8) * 4096;

            auto &prev = sats[m->svprn];

            if (prev.svprn != m->svprn || prev.week != m->issue_week || prev.time_of_week != toa || prev.almanac != m->data) {
                if (verbose) {
                    std::cout << "# Sat #" << m->svprn
                              << " (" << sats.size() << "/" << sats_needed << ")"
                              << " [" << m->issue_week << "." << toa << " -> " << to_hex(m->data) << "]"
                              << std::endl;
                }

                sats[m->svprn] = { m->svprn, m->issue_week, toa, m->data };

                if (check_sats(sats)) {
                    std::string hash = hash_almanac(sats);

                    if (hash != curr_rand) {
                        curr_rand = hash;
                        time_t t = decode_gps_time(m->issue_week, toa);

                        std::cout << "rand " << to_hex(curr_rand)
                                  << " " << m->issue_week << "." << toa
                                  << " " << t
                                  << std::endl;

                        if (!loop) std::exit(0);
                    }
                }
            }
        }
    }
}

}
