#include <iostream>

#include <docopt/docopt.h>

#include "ephemerand/gps_utils.h"
#include "ephemerand/util.h"



namespace ephemerand {
    void cmd_run(std::string device, bool verbose, bool loop);
    void cmd_reset(std::string device, bool verbose);
};


static const char USAGE[] =
R"(ephemerand - global randomness beacon

    Usage:
      ephemerand run [--device=<device>] [--verbose] [--loop]
      ephemerand reset [--device=<device>] [--verbose]
      ephemerand decode-gps-time <week> <time_of_week>
      ephemerand (-h | --help)
      ephemerand --version

    Options:
      -h --help
      --version
      --device=<device>  Serial device to use (default: /dev/ttyACM0)
      --verbose          Print extra information (default: false)
      --loop             Don't exit. Print new random values as they are generated (default: false)
)";


int parse_command_line(int argc, char **argv) {
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, { argv + 1, argv + argc }, true, "ephemerand 0.0.1");

    std::string device = "/dev/ttyACM0";
    if (args["--device"]) device = args["--device"].asString();

    bool verbose = false;
    if (args["--verbose"]) verbose = args["--verbose"].asBool();

    if (args["run"].asBool()) {
        bool loop = false;
        if (args["--loop"]) loop = args["--loop"].asBool();

        ephemerand::cmd_run(device, verbose, loop);
    } else if (args["reset"].asBool()) {
        ephemerand::cmd_reset(device, verbose);
    } else if (args["decode-gps-time"].asBool()) {
        time_t t = decode_gps_time(args["<week>"].asLong(), args["<time_of_week>"].asLong());
        std::cout << t << std::endl;
    } else {
        throw ephemerand::error("unrecognized command");
    }

    return 0;
}


int main(int argc, char **argv) {
    try {
        parse_command_line(argc, argv);
    } catch (std::exception &e) {
        std::cerr << "CAUGHT EXCEPTION, ABORTING: " << e.what() << std::endl;
        ::exit(1);
    }

    return 0;
}
