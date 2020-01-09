#include "motorControl.h"

void Kangaroo::openSerialPort() {
    // open the port
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if(fd < 0) {
        std::cerr << "[ERROR]: Unable to open port " << portName << std::endl;
    } else {
        std::cout << "[INFO]: Port " << fd << " is open!" << std::endl;
    }

}

void Kangaroo::configureSerialPort() {
    struct termios SerialPortSettings; // Create the structure
    memset(&SerialPortSettings, 0, sizeof SerialPortSettings);

    tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port

    SerialPortSettings.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    SerialPortSettings.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    SerialPortSettings.c_cflag |= CS8; // 8 bits per byte (most common)
    SerialPortSettings.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    SerialPortSettings.c_lflag &= ~ICANON;
    SerialPortSettings.c_lflag &= ~ECHO; // Disable echo
    SerialPortSettings.c_lflag &= ~ECHOE; // Disable erasure
    SerialPortSettings.c_lflag &= ~ECHONL; // Disable new-line echo
    SerialPortSettings.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    SerialPortSettings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes

    SerialPortSettings.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    SerialPortSettings.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    SerialPortSettings.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    SerialPortSettings.c_cc[VMIN] = 0;

    cfsetispeed(&SerialPortSettings,baudRate); // Set Read  Speed
    cfsetospeed(&SerialPortSettings,baudRate); // Set Write Speed


    if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
        std::cerr << "[ERROR]: Unable to configure port " << portName << std::endl;
    else
        std::cout << "[INFO]: BaudRate = " << baudRate << ", StopBits = 1, Parity = none" << std::endl;

}

void Kangaroo::closeSerialPort() {
    close(fd);
    std::cout << "[INFO]: Port " << portName << " is closed!" << std::endl;
}

void Kangaroo::startMotor(char c) {
    char msg[] = {c, ',', 's', 't', 'a', 'r', 't', '\r'};
    write(fd, msg, sizeof(msg));
}


void Kangaroo::getPosition(char c) {
    char buffer[9];
    //memset(&buffer, '\0', sizeof(buffer));
    char getmsgx[] = {c, ',', 'g', 'e', 't', 'p', '\r'};

    write(fd, getmsgx, sizeof(getmsgx));
    usleep(20000);
    ssize_t bytes = read(fd, &buffer, sizeof(buffer));

    //std::cout << "[M" << c << "] Returned number of bytes: " << bytes << std::endl;
    std::cout << "[GETPOS]: Port " << fd << ", Motor " << c << ": " << buffer << std::endl;

}


void Kangaroo::setPosition(char c) {
    char msg[] = {c, ',', 'p', '3', '0', '0', '0', '\r'};
    write(fd, msg, sizeof(msg));

    std::cout << "[SETPOS]: Port " << fd << ", Motor " << c << ": " << msg << std::endl;

}

void Kangaroo::homePosition(char c) {
    char msg[] = {c, ',', 'p', '2', '5', '0', '0', '\r'};
    write(fd, msg, sizeof(msg));

    std::cout << "[HOMPOS]: Port " << fd << ", Motor " << c << ": " << msg << std::endl;

}



