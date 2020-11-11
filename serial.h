#ifndef SERIAL_H
#define SERIAL_H

// Used for serial
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <stdio.h>
#include <errno.h>
#include <string>
#include <cstring>

class Serial{

public:

    Serial( int baud = B115200 );

    bool read_char( unsigned char * c );
    bool read_chars( unsigned char * buf, int len );

private:

    int port_id;
    struct termios port_settings;

};

#endif // SERIAL_H
