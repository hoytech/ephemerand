#include <string>
#include <map>

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

            if (check_sats(sats)) std::cout << "GOOD TO GO" << std::endl;

/*
            uint32_t *words = reinterpret_cast<uint32_t*>(m->data.data());
            double e = (double)(words[0] & 0xFFFF) * P2_21;
            double sqrta = (double)(words[3] & 0xFFFFFF) * P2_11;
            double deltai = (double)(words[1] & 0xFFFF) * P2_19 * SC2RAD;
            double i0 = 0.3*SC2RAD + deltai;

            std::cout << "  e = " << e << "  sqrta = " << sqrta << " i0 = " << i0 << std::endl;
*/
/*
    auto buff = m->data.data();
    double e = getbitu(buff,32,16)*P2_21;
    double toas = getbitu(buff,16, 8)*4096.0;
    std::cout << ": e = " << e << " toas = " << toas << std::endl;
*/
        }
    }
}

}
