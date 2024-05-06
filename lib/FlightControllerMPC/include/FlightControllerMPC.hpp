#ifndef FLIGHT_CONTROLLER_MPC_H
#define FLIGHT_CONTROLLER_MPC_H

#include <IMU.h>

IMU imu;

struct motorData {
    double omega_1;
    double omega_2;
    double omega_3;
    //double alpha;
};

class FlightController {
public:
    FlightController(double dt, bool spinning = false);
    motorData calculate();
private:
    double dt;
    motorData motor;
};

#endif
