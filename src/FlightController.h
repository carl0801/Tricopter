#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <IMU.h>
#include <cmath>
#include <algorithm>
#include <ESP32Servo.h>
#include <ESP32PWM.h>




struct motorData {
    double omega_1;
    double omega_2;
    double omega_3;
    double alpha;
};

//void updateMotor(double omega_1, double omega_2, double omega_3, double alpha);

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
    PIDController(double kp, double ki, double kd, double dt=0.01, double max_integral=0.0, double derivative_filter=0.1);

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
public:
    FlightController(double dt);

    motorData calculate();
private:
    double dt;
    PIDController TransControlZ;
    PIDController RotControlZ;
    PIDController RotControlX;
    PIDController RotControlY;
    Tricopter drone;
    motorData Output;

    // Member variables
    double target_z;
    double target_roll;
    double target_pitch;
    double target_yaw;

    double z;
    double roll;
    double pitch;
    double yaw;

    Eigen::Quaterniond q;

    double z_error;
    double U_z;
    double roll_error;
    double U_r;
    double pitch_error;
    double U_p;
    double yaw_error;
    double U_y;

    double omega_1;
    double omega_2;
    double omega_3;
    double alpha;

    double term1_12;
    double term2_12;
    double term3_12;
    double omega_1_mid;
    double omega_2_mid;

    long double term1_3p1;
    long double term1_3p2;
    long double term1_3;

    long double term2_3p1;
    long double term2_3p2;
    long double term2_3;

    double alpha_term1;
    double alpha_term2;


};

#endif // FLIGHTCONTROLLER_H
