#ifndef FLIGHTCONTROLLER_H
#define FLIGHTCONTROLLER_H

#include <IMU.h>
#include <cmath>
#include <algorithm>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <ArduinoEigen.h>



struct motorData {
    double omega_1;
    double omega_2;
    double omega_3;
    double alpha_1;
    double alpha_2;
    double alpha_3;
};

//void updateMotor(double omega_1, double omega_2, double omega_3, double alpha);

void resetTargetAngle(double& yaw, double& x, double& y, double& z);


class Tricopter {
public:
    // Constructor
    Tricopter(double mass, double l_0, double gravity, double drag, double j_x, double j_y, double j_z, double k_t1, double k_t2, double k_t3, double k_d);
    
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
    double k_t1;
    double k_t2;
    double k_t3;
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
    IMU imu;

    motorData calculate(double yawOffset);
private:
    double dt;
    PIDController TransControlX;
    PIDController TransControlY;
    PIDController TransControlZ;
    const Eigen::Matrix3d pquad;
    const Eigen::Matrix3d pquad2;
    //PIDController RotControlZ;
    //PIDController RotControlX;
    //PIDController RotControlY;
    const Tricopter drone;
    motorData Output;
    const Eigen::Matrix<double, 6, 6> M;
    Eigen::Matrix<double, 6, 1> U;
    Eigen::Matrix<double, 6, 1> Omega;
    // Member variables
    double target_x;
    double target_y;
    double target_z;
    
    Eigen::Quaterniond target_q;

    double x;
    double y;
    double z;
    double yaw;
    double roll;
    double pitch;
    double w, qx, qy, qz;
    double lidar2;

    Eigen::Quaterniond q;
    Eigen::Vector3d angular_velocity;
    
    double x_error;
    double y_error;
    double z_error;

    Eigen::Quaterniond quad_error;
    Eigen::Vector3d U_quad;
};

#endif // FLIGHTCONTROLLER_H
