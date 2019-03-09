#include <hoytech/protected_queue.h>

#include "ephemerand/ublox.h"
#include "ephemerand/util.h"


namespace ephemerand {


void cmd_reset(std::string device, bool verbose) {
    ephemerand::Ublox ub(device);

    ub.connect();
    ub.run();


    while(1) {
        auto msg = ub.output_queue.pop();

        if (auto m = std::get_if<UbloxMessage_Version>(&msg)) {
            if (verbose) std::cout << "# Connection OK. SW: " << m->software_version << " HW: " << m->hardware_version << std::endl;
            ub.reset();
        } else if (auto m = std::get_if<UbloxMessage_Ack>(&msg)) {
            if (m->message_class == MSG_CLASS_CFG && m->message_id == MSG_ID_CFG_RST) {
                if (verbose) std::cout << "# Device has been reset." << std::endl;
                std::exit(0);
            }

            std::cerr << "Unexpected ack: " << m->message_class << " / " << m->message_id << std::endl;
            std::exit(1);
        } else {
            std::cerr << "Unrecognized message type" << std::endl;
            std::exit(1);
        }
    }
}

}
