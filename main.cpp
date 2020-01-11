#include <chrono>
#include <cmath>
#include "motorControl.h"
#define EPS 5 // position error in mV
#define deg2rad M_PI/180;


using namespace std::chrono;


int main(){
    // generate a simple sinusoidal trajectory
    int size = 40;
    int posM1[size];
    int posM2[size];
    int posM3[size];

    //phase shift, amplitudes and angular velocity
    double phase = 120*deg2rad;
    double amp1 = 2000.0;
    double amp2 = 2000.0;
    double amp3 = 2000.0;
    double w = 1;

    // home position
    int homePosition = 2500;

    //generate motor positions
    for(int it = 0; it!= size;++it){
        posM1[it] = -int(amp1*sin(w*it*M_PI/size)) + homePosition;
        posM2[it] = int(amp2*sin(w*it*M_PI/size)) + homePosition;
        posM3[it] = int(amp3*sin(w*it*M_PI/size)) + homePosition;

    }

    // define ports and baud rates
    std::string port1("/dev/ttyUSB0");
    speed_t br1 = B19200;

    std::string port2("/dev/ttyUSB1");
    speed_t br2 = B19200;

    // motor IDs
    char M1 = '1', M2 = '2';

    // Connect to Kangaroo controllers
    // Controller 1
    Kangaroo kangaroo1(port1, br1);
    kangaroo1.openSerialPort();
    kangaroo1.configureSerialPort();

    // Controller 2
    Kangaroo kangaroo2(port2, br2);
    kangaroo2.openSerialPort();
    kangaroo2.configureSerialPort();

    // start driver 1 motor 1
    kangaroo1.startMotor(M1);
    kangaroo1.getPosition(M1);

    // start driver 2 motor 1 and 2
    kangaroo2.startMotor(M1);
    kangaroo2.startMotor(M2);

    // move to the home position
    kangaroo1.homePosition(M1);
    kangaroo2.homePosition(M1);
    kangaroo2.homePosition(M2);
    usleep(2000000);

    // move to the starting position
    kangaroo1.setPosition(M1, posM1[0]);
    kangaroo2.setPosition(M1, posM2[0]);
    kangaroo2.setPosition(M2, posM3[0]);
    usleep(2000000);

    // get positions
    kangaroo2.getPosition(M1);
    kangaroo1.getPosition(M1);
    kangaroo2.getPosition(M2);

    int counter = 1;
    while(counter < size){
        std:: cout << "Iteration: " << counter << "-------------" << std::endl;
        // set positions
        kangaroo2.setPosition(M1, posM2[counter]);
        kangaroo1.setPosition(M1, posM1[counter]);
        kangaroo2.setPosition(M2, posM3[counter]);

        usleep(100000);
        // get positions
        kangaroo2.getPosition(M1);
        kangaroo1.getPosition(M1);
        kangaroo2.getPosition(M2);

        // get speeds
        kangaroo2.getSpeed(M1);
        kangaroo1.getSpeed(M1);
        kangaroo2.getSpeed(M2);

        ++counter;
    }

    kangaroo1.homePosition(M1);
    kangaroo2.homePosition(M1);
    kangaroo2.homePosition(M2);

    usleep(2000000);
    kangaroo1.getPosition(M1);
    kangaroo2.getPosition(M1);
    kangaroo2.getPosition(M2);

    kangaroo1.closeSerialPort();
    kangaroo2.closeSerialPort();

    return 0;
}
