//
// Created by denis on 1/6/20.
//

#ifndef MOTORCONTROL_MOTORCONTROL_H
#define MOTORCONTROL_MOTORCONTROL_H

#include <iostream>
// C library headers
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class Kangaroo{
public:
    // constructors
    Kangaroo() = default;
    Kangaroo(std::string &pn, speed_t &br){
        portName = pn;
        baudRate = br;
    }

    // open the serial port
    void openSerialPort();

    // configure the serial port
    void configureSerialPort();

    // close the serial port
    void closeSerialPort();

    // start motors
    void startMotor(char c);

    // get positions
    void getPosition(char c);

    // set positions
    void setPosition(char c);

    // home position
    void homePosition(char c);

private:
    std::string portName = "/dev/ttyUSB0";
    speed_t baudRate = B19200;
    int fd = 0; // // file description for the serial port
};

#endif //MOTORCONTROL_MOTORCONTROL_H
