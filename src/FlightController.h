#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <IMU.h>
#include <cmath>
#include <algorithm>
#include <ESP32Servo.h>
#include <ESP32PWM.h>



void updateMotor(double omega_1, double omega_2, double omega_3, double alpha);

void resetTargetAngle(double& roll, double& pitch, double& yaw, double& z);

class Tricopter {
public:
    // Constructor
    Tricopter(double mass, double l_0, double gravity, double drag, double j_x, double j_y, double j_z, double k_t, double k_d);
    
    // Member variables
    double mass;
    double l_0;
    double l_1;
    double l_2;
    double gravity;
    double drag;
    double j_x;
    double j_y;
    double j_z;
    double k_t;
    double k_d;
};

class PIDController {
public:
    // Constructor
    PIDController(double kp, double ki, double kd, double dt=0.01, double max_integral=0.0, double derivative_filter=1.5);

    // Member function to calculate PID output
    double calculate(double error);

private:
    // Member variables
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
    double prev_derivative;
    double dt;
    double max_integral;
    double derivative_filter;
};

class FlightController {
private:
    double dt;
    PIDController TransControlZ;
    PIDController RotControlZ;
    PIDController RotControlX;
    PIDController RotControlY;
    Tricopter drone;

    // Member variables
    double target_z;
    double target_roll;
    double target_pitch;
    double target_yaw;

    double z;
    double roll;
    double pitch;
    double yaw;

    double omega_1;
    double omega_2;
    double omega_3;
    double alpha;

public:
    FlightController(double dt);

    std::tuple<double, double, double, double> calculate();
};

#endif // FLIGHTCONTROLLER_H
