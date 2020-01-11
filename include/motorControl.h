//
// Created by askat on 1/6/20.
//

#ifndef MOTORCONTROL_MOTORCONTROL_H
#define MOTORCONTROL_MOTORCONTROL_H

#include <iostream>
#include <utility>
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
    void startMotor(char &motorID);

    // get motor position
    std::pair<int, bool> getPosition(char &motorID);

    // get motor speed
    std::pair<int, bool> getSpeed(char &motorID);

    // set a reference position
    void setPosition(char &motorID, int &pos);

    // set a reference position and speed
    void setPositionSpeed(char &motorID, int &pos, int &speed);

    // home position
    void homePosition(char &motorID);

private:
    std::string portName = "/dev/ttyUSB0";
    speed_t baudRate = B19200;
    int fd = 0; // // file description for the serial port
    useconds_t readTime = 25000; // time to read position (microsecond)
};

#endif //MOTORCONTROL_MOTORCONTROL_H
