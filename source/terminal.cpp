//
// Created by ritn on 10/20/23.
//
#include "../include/terminal.h"

#include <vector>
#include <thread>

terminal::terminal() {
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
}

terminal::terminal(const tty_type::tty_dev& dev) {
    this->tty_device = dev;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
}

terminal::terminal(const tty_type::tty_rate &rate) {
    this->tty_baud_rate = rate;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, rate);
    cfsetospeed(&tty, rate);
}

terminal::terminal(const tty_type::tty_dev &dev, const tty_type::tty_rate &rate) {
    this->tty_baud_rate = rate;
    this->tty_device    = dev;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, rate);
    cfsetospeed(&tty, rate);
}

terminal::terminal(const terminal &term) {
    this->tty           = term.tty;
    this->tty_baud_rate = term.tty_baud_rate;
    this->tty_device    = term.tty_device;
    this->tty_fd        = term.tty_fd;

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    cfsetispeed(&tty, this->tty_baud_rate);
    cfsetospeed(&tty, this->tty_baud_rate);
}

terminal::tty_errno terminal::serial() {
    if (this->tty_device.size() <= 0)
        return tty_errno::ttyOpenError;

    this->tty_fd = open(this->tty_device.c_str(), O_RDWR);

    if (this->tty_fd < 0)
        return tty_errno::ttyOpenError;

    if (tcgetattr(this->tty_fd, &tty) != 0)
        return tty_errno::ttyTCGetAddrError;

    if (tcsetattr(this->tty_fd, TCSANOW, &tty) != 0)
        return tty_errno::ttyTCGetAddrTCSANOWError;

    std::vector<char> read_buffer(__INT16_MAX__);
    std::string       send_buffer;

    auto read_handler([&]() -> void {
        int32_t bytes_read = 0;

        while (bytes_read >= 0) {
            bytes_read = read(this->tty_fd, read_buffer.data(), __INT16_MAX__);

            std::cout << read_buffer.data() << std::endl;

            read_buffer.clear();
            read_buffer.resize(__INT16_MAX__);
        }

        std::cout << "Error reading data" << std::endl;
    });

    std::thread(read_handler).detach();

    while (true) {
        std::cout << "send> ";
        std::cin  >> send_buffer;

        write(this->tty_fd, send_buffer.c_str(), send_buffer.size());
    }
}

void terminal::setTTYBaudRate(const tty_type::tty_rate &rate) {
    switch (rate) {
        case B115200:
            this->tty_baud_rate = B115200;
            break;
        case B9600:
            this->tty_baud_rate = B9600;
            break;
        case B1000000:
            this->tty_baud_rate = B1000000;
            break;
        case B1152000:
            this->tty_baud_rate = B1152000;
            break;
        default:
            std::cout << "Mode doesn't support" << std::endl;
            break;
    }
}

void terminal::setTTYDevice(const tty_type::tty_dev &dev) {
    this->tty_device = dev;
}

void terminal::setTTYPort(const tty_type::tty_port &port) {
    this->tty_fd = port;
}

void terminal::setTTYTerminal(const tty_type::tty_terminal &ttyTerminal) {
    this->tty = ttyTerminal;
}

terminal::~terminal() {
    close(this->tty_fd);
}