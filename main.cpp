#include "motorControl.h"

int main(){
    std::string port1("/dev/ttyUSB0");
    speed_t br1 = B19200;

    std::string port2("/dev/ttyUSB1");
    speed_t br2 = B19200;

    // 1 Motor
    Kangaroo motor1(port1, br1);
    motor1.openSerialPort();
    motor1.configureSerialPort();

    motor1.startMotor('1');
    motor1.getPosition('1');
    /*motor1.setPosition('1');
    usleep(1000000);
    motor1.getPosition('1');
    motor1.homePosition('1');
    usleep(1000000);
    motor1.getPosition('1');*/

    // 2 Motors
    Kangaroo motor2(port2, br2);
    motor2.openSerialPort();
    motor2.configureSerialPort();

    motor2.startMotor('1');
    motor2.startMotor('2');

    motor2.getPosition('1');
    motor2.getPosition('2');

    /*motor2.setPosition('1');
    motor2.setPosition('2');

    usleep(1000000);

    motor2.setPosition('1');
    motor2.setPosition('2');

    motor2.homePosition('1');
    motor2.homePosition('2');
    usleep(1000000);
    motor2.getPosition('1');
    motor2.getPosition('2');*/

    motor1.closeSerialPort();
    motor2.closeSerialPort();


    return 0;
}