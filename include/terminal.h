//
// Created by ritn on 10/20/23.
//

#ifndef SERIALCONSOLE_TERMINAL_H
#define SERIALCONSOLE_TERMINAL_H

#include <iostream>
#include <string>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

namespace tty_type {
    typedef termios tty_terminal;
    typedef int32_t     tty_port;
    typedef std::string  tty_dev;
    typedef u_int32_t   tty_rate;
}

class terminal {
private:
    tty_type::tty_terminal       tty;
    tty_type::tty_port        tty_fd;
    tty_type::tty_dev     tty_device;
    tty_type::tty_rate tty_baud_rate;

public:
    enum class tty_errno
            : u_int32_t {
        ttyOpenError             = 0xa1,
        ttyTCGetAddrError        = 0xa2,
        ttyTCGetAddrTCSANOWError = 0xa3,
        ttyReadError             = 0xa4,
        ttyWriteError            = 0xa5
    };

    terminal();
    terminal(const tty_type::tty_dev&);
    terminal(const tty_type::tty_rate&);
    terminal(const tty_type::tty_dev&, const tty_type::tty_rate&);
    terminal(const terminal&);

    void setTTYDevice(const tty_type::tty_dev&);
    void setTTYTerminal(const tty_type::tty_terminal&);
    void setTTYBaudRate(const tty_type::tty_rate&);
    void setTTYPort(const tty_type::tty_port&);

    [[nodiscard]] tty_type::tty_dev        getTTYDevice() const { return this->tty_device;    }
    [[nodiscard]] tty_type::tty_terminal getTTYTerminal() const { return this->tty;           }
    [[nodiscard]] tty_type::tty_port         getTTYPort() const { return this->tty_fd;        }
    [[nodiscard]] tty_type::tty_rate         getTTYRate() const { return this->tty_baud_rate; }

    tty_errno serial();

    ~terminal();
};

#endif //SERIALCONSOLE_TERMINAL_H
