// g++ GNSS.cpp -L./build -lminmea -o GNSS

#include <iostream>
#include <fstream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>

extern "C" {
    #include "minmea.h"
}

int configure_serial(int fd) {
    termios tty{};
    if (tcgetattr(fd, &tty) != 0) return -1;
    cfsetospeed(&tty, B115200); cfsetispeed(&tty, B115200);
    tty.c_cflag = CLOCAL | CREAD | CS8;
    tty.c_lflag = tty.c_iflag = tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 10;
    return tcsetattr(fd, TCSANOW, &tty);
}

int main() {
    int fd = open("/dev/ttyACM1", O_RDONLY | O_NOCTTY);
    if (fd < 0 || configure_serial(fd) != 0) return 1;

    std::string line;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') {
            const char* buf = line.c_str();
            switch (minmea_sentence_id(buf, false)) {
                case MINMEA_SENTENCE_GGA: {
                    minmea_sentence_gga gga{};
                    if (minmea_parse_gga(&gga, buf)) {
                        std::cout << "GGA lat=" << minmea_tocoord(&gga.latitude)
                                  << " lon=" << minmea_tocoord(&gga.longitude)
                                  << " sats=" << gga.satellites_tracked << '\n';
                    }
                    break;
                }
                case MINMEA_SENTENCE_RMC: {
                    minmea_sentence_rmc rmc{};
                    if (minmea_parse_rmc(&rmc, buf)) {
                        std::cout << "RMC lat=" << minmea_tocoord(&rmc.latitude)
                                  << " lon=" << minmea_tocoord(&rmc.longitude)
                                  << " speed=" << minmea_tofloat(&rmc.speed) * 1.852 << " km/h\n";
                    }
                    break;
                }
                default: break;
            }
            line.clear();
        } else {
            line += c;
        }
    }

    close(fd);
    return 0;
}
