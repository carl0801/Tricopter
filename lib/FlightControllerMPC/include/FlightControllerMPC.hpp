#ifndef FLIGHT_CONTROLLER_MPC_H
#define FLIGHT_CONTROLLER_MPC_H

//#include <IMU.h>

//IMU imu;

struct motorData {
    double force_1;
    double force_2;
    double force_3;
    bool tilt;
};

class FlightController {
public:
    FlightController(double dt);
    motorData calculate();
private:
    double dt;
    motorData motor;
};

#endif
