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

class PIDController3D {
public:
    PIDController3D(const Eigen::Vector3d& kp, const Eigen::Vector3d& ki, const Eigen::Vector3d& kd, double dt, const Eigen::Vector3d& max_integral, double derivative_filter)
        : kp(kp), ki(ki), kd(kd), dt(dt), max_integral(max_integral), derivative_filter(derivative_filter),
          integral(Eigen::Vector3d::Zero()), prev_error(Eigen::Vector3d::Zero()), prev_derivative(Eigen::Vector3d::Zero()) {}

    Eigen::Vector3d calculate(const Eigen::Vector3d& error) {
        // Update integral term
        integral += error * dt;
        
        // Anti-windup
        for (int i = 0; i < 3; ++i) {
            if (max_integral[i] != 0.0) {
                integral[i] = std::max(std::min(integral[i], max_integral[i]), -max_integral[i]);
            }
        }
            
        // Calculate derivative term
        Eigen::Vector3d derivative = (error - prev_error) / dt;
        
        // Apply derivative filter
        if (derivative_filter != 0.0) {
            derivative = prev_derivative * (1 - derivative_filter) + derivative * derivative_filter;
        }
        
        // Calculate PID output
        Eigen::Vector3d output = kp.cwiseProduct(error) + ki.cwiseProduct(integral) + kd.cwiseProduct(derivative);
        
        // Update previous error and derivative
        prev_error = error;
        prev_derivative = derivative;
        
        return output;
    }

private:
    Eigen::Vector3d kp, ki, kd;
    double dt;
    Eigen::Vector3d max_integral;
    double derivative_filter;
    Eigen::Vector3d integral;
    Eigen::Vector3d prev_error;
    Eigen::Vector3d prev_derivative;
};

class FlightController {
public:
    FlightController(double dt);
    IMU imu;

    motorData calculate(double yawOffset,Eigen::Quaterniond target_q, double target_x, double target_y, double target_z);
private:
    double dt;
    PIDController TransControlX;
    PIDController TransControlY;
    PIDController TransControlZ;
    PIDController3D OrientationControl;
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
    Eigen::Vector3d error_vector;
    
    double x_error;
    double y_error;
    double z_error;

    Eigen::Quaterniond quad_error;
    Eigen::Vector3d U_quad;
};




#endif // FLIGHTCONTROLLER_H
