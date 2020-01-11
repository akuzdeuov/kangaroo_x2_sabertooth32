#include "motorControl.h"

void Kangaroo::openSerialPort() {
    // open the port
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

    if(fd < 0) {
        std::cerr << "[ERROR] Unable to open the port " << portName << std::endl;
    } else {
        std::cout << "[INFO] Port " << fd << " was successfully opened!" << std::endl;
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
        std::cerr << "[ERROR] Unable to configure port " << portName << std::endl;
    else
        std::cout << "[INFO] BaudRate = " << baudRate << ", StopBits = 1, Parity = none" << std::endl;

}

void Kangaroo::closeSerialPort() {
    close(fd);
    std::cout << "[INFO] Port " << portName << " was successfully closed!" << std::endl;
}

void Kangaroo::startMotor(char &motorID) {
    char msg[] = {motorID, ',', 's', 't', 'a', 'r', 't', '\r'};
    write(fd, msg, sizeof(msg));
}

std::pair<int, bool> Kangaroo::getPosition(char &motorID) {
    char readBuffer[9];
    //memset(&buffer, '\0', sizeof(buffer));
    char getMsg[] = {motorID, ',', 'g', 'e', 't', 'p', '\r'};
    write(fd, getMsg, sizeof(getMsg));
    usleep(readTime);
    // read buffer
    ssize_t readBytes = read(fd, &readBuffer, sizeof(readBuffer));
    // extract numerical token
    std::string str;
    for( ssize_t ind = 3; ind < readBytes; ++ind){
        //std::cout << buffer[ind];
        str += readBuffer[ind];
    }

    // container for position info (current position, status)
    std::pair<int, bool> positionStatus;
    // current position
    positionStatus.first = std::stoi(str);
    // check whether the motor is moving or not
    positionStatus.second = readBuffer[2] == 'P';
    // print position info
    std::cout << "[GETPOS] Port: " << fd << ", Motor: " << motorID << ", Position: " << positionStatus.first << " mV, Status: ";
    std::cout << (positionStatus.second ? "Not Moving" : "Moving") << std::endl;

    return positionStatus;
}

std::pair<int, bool> Kangaroo::getSpeed(char &motorID) {
    char readBuffer[9];
    //memset(&buffer, '\0', sizeof(buffer));
    char getMsg[] = {motorID, ',', 'g', 'e', 't', 's', '\r'};
    write(fd, getMsg, sizeof(getMsg));
    usleep(readTime);
    // read buffer
    ssize_t readBytes = read(fd, &readBuffer, sizeof(readBuffer));
    // extract numerical token
    std::string str;
    for( ssize_t ind = 3; ind < readBytes; ++ind){
        str += readBuffer[ind];
    }
    // container for speed info (current speed, status)
    std::pair<int, bool> speedStatus;
    // current speed
    speedStatus.first = std::stoi(str);
    // check whether the speed is max or still accelerating
    speedStatus.second = readBuffer[2] == 'S';
    // print speed info
    std::cout << "[GETSPD] Port: " << fd << ", Motor: " << motorID << ", Speed: " << speedStatus.first << " mV, Status: ";
    std::cout << (speedStatus.second ? "Max Speed" : "Accelerating") << std::endl;

    return speedStatus;

}


void Kangaroo::setPosition(char &motorID, int &pos) {
    // identify motor to send command
    std::string str1;
    if(motorID == '1'){
        str1 = "1,p";
    }else if(motorID == '2'){
        str1 = "2,p";
    }
    // convert integer to string
    std::string str2 = std::to_string(pos);
    // check the command
    switch(str2.length()){
        case 1:
            str2 = "000" + str2;
            break;
        case 2:
            str2 = "00" + str2;
            break;
        case 3:
            str2 = "0" + str2;
            break;
        case 4:
            str2 = str2;
            break;
        default:
            str2 = "0000";
            break;
    }

    // construct the final command
    std::string cmdStr = str1 + str2 + '\r';

    //char msg[] = {motorID, ',', 'p', '0', '9', '0', '0', '\r'};
    // convert string to char
    char const *chCmd = cmdStr.c_str();
    // send the command to the motor
    write(fd, chCmd, sizeof(chCmd));
    // print the sent command
    std::cout << "[SETPOS] Port " << fd << ", Motor: " << motorID << ": Reference Position: " << pos << " mV" << std::endl;

}

void Kangaroo::setPositionSpeed(char &motorID, int &pos, int &speed) {
    // 1,p1000 s200
    // identify motor to send command
    std::string init;
    if(motorID == '1'){
        init = "1,p";
    }else if(motorID == '2'){
        init = "2,p";
    }

    std::string posStr = std::to_string(pos);
    std::string spdStr = std::to_string(speed);

    // construct the final command
    std::string cmdStr = init + posStr + spdStr + '\r';

    //char msg[] = {motorID, ',', 'p', '0', '9', '0', '0', '\r'};
    // convert string to char
    char const *chCmd = cmdStr.c_str();
    // send the command to the motor
    write(fd, chCmd, sizeof(chCmd));
    // print the sent command
    std::cout << "[SETPOS&SPD] Port " << fd << ", Motor: " << motorID
              <<": Reference Position and Speed: " << pos << " " << speed << " mV" << std::endl;

}

void Kangaroo::homePosition(char &motorID) {
    char msg[] = {motorID, ',', 'p', '2', '5', '0', '0', '\r'};
    write(fd, msg, sizeof(msg));

    std::cout << "[HOMPOS] Port: " << fd << ", Motor: " << motorID << ": Home Position: " << msg << std::endl;

}



