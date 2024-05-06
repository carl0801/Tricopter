#include <Arduino.h>
#include <ArduinoEigen.h>
#include "FlightController.h"
#include "ipmmpc.hpp"

using namespace Eigen;

// FlightController class constructor
FlightController::FlightController(double dt) : dt(dt) {}

motorData FlightController::calculate() {
    // Initial state
    Vector3f x0(0, 0, 0);
    Vector3f p0(0, 0, 0);
    Vector3f v0(0, 0, 0);
    Vector3f w0(0, 0, 0);
    Matrix3f R = Matrix3f::Identity();
    // Target position
    Vector3f pt(0, 0.2, 0.5);
    // Call Interior Point Method
    Vector3f x = IPMMPC(x0, pt, p0, v0, w0, R);
    // Update motorData
    motor.omega_1 = x(0);
    motor.omega_2 = x(1);
    motor.omega_3 = x(2);

    return motor;
}
