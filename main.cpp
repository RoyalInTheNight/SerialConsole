//
// Created by ritn on 10/20/23.
//
#include "include/terminal.h"

void usage(const char *cmd_arg) {
    std::cout << "Usage: " << cmd_arg << " <device>  <speed>" << std::endl;
}

int32_t main(int32_t argc, char **argv) {
    if (argc < 3) {
        usage(argv[0]);
        return -1;
    }

    std::string dev   = argv[1];
    std::string speed = argv[2];

    terminal serial(dev);

    if (speed == "115200")
        serial.setTTYBaudRate(B115200);

    if (speed == "9600")
        serial.setTTYBaudRate(B9600);

    if (speed == "1000000")
        serial.setTTYBaudRate(B1000000);

    if (speed == "1152000")
        serial.setTTYBaudRate(B1152000);

    serial.serial();

    return 0;
}