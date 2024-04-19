#include <Arduino.h>
#include "FlightController.h"


void resetTargetAngle(Eigen::Quaterniond& q, double& x, double& y, double& z) {
    q = Eigen::Quaterniond(1, 0, 0, 0);
    x = 0.0;
    y = 0.0;
    z = 0.0;
}


Tricopter::Tricopter(double mass, double l_0, double gravity, double drag, double j_x, double j_y, double j_z, double k_t, double k_d) :
    mass(mass), l_0(l_0), gravity(gravity), drag(drag), j_x(j_x), j_y(j_y), j_z(j_z), k_t(k_t), k_d(k_d) {
        //take the abs value of the sin function to get the length of the arms
        l_1 = abs(sin(60 * M_PI / 180) * l_0);
        l_2 = abs(cos(60 * M_PI / 180) * l_0);
    }

PIDController::PIDController(double kp, double ki, double kd, double dt, double max_integral, double derivative_filter) :
    kp(kp), ki(ki), kd(kd), integral(0), prev_error(0), prev_derivative(0), dt(dt), max_integral(max_integral), derivative_filter(derivative_filter) {}

double PIDController::calculate(double error) {
        // Update integral term
        integral += error * dt;
        
        // Anti-windup
        if (max_integral != 0.0) {
            integral = std::max(std::min(integral, max_integral), -max_integral);
        }
            
        // Calculate derivative term
        double derivative = (error - prev_error) / dt;
        
        // Apply derivative filter
        if (derivative_filter != 0.0) {
            derivative = prev_derivative * (1 - derivative_filter) + derivative * derivative_filter;
        }
        
        // Calculate PID output
        double output = kp * error + ki * integral + kd * derivative;
        
        // Update previous error and derivative
        prev_error = error;
        prev_derivative = derivative;
        
        return output;
}

FlightController::FlightController(double dt) : dt(dt),
    imu(dt),
    TransControlX(6,1,1, dt),
    TransControlY(6,1,1, dt),
    TransControlZ(2,0,0, dt),
    //pquad is a 3x3 diagonal matrix with 0.1 in the first diagonal slot, 10 in the middle and 1 in the last
    //pquad((Eigen::Vector3d(0.1, 10, 1)).asDiagonal()),
    pquad((Eigen::Vector3d(0.1, 1, 0.1)).asDiagonal()),
    //iden is a 3x3 identity matrix * 0.01
    pquad2(Eigen::Matrix3d::Identity() * 0.01),
    //3119363164318645 wheight
    drone(0.479, 0.33, 9.81, 0.02, 0.035, 0.035, 0.02, 6.769e-6, 7.295e-8),
    //  1                                                               2                                                   3                                                 4                                         5                           6        
    M((Eigen::Matrix<double, 6, 6>() << 
        -std::sqrt(3)/(3*drone.k_t),                                    1/(3*drone.k_t),                                    -drone.k_d/(3*drone.l_0*std::pow(drone.k_t,2)),   0,                                        0,                          1/(3*drone.l_0*drone.k_t),
        std::sqrt(3)/(3*drone.k_t),                                     1/(3*drone.k_t),                                    -drone.k_d/(3*drone.l_0*std::pow(drone.k_t,2)),   0,                                        0,                          1/(3*drone.l_0*drone.k_t),
        0,                                                              -2/(3*drone.k_t),                                   -drone.k_d/(3*drone.l_0*std::pow(drone.k_t,2)),   0,                                        0,                          1/(3*drone.l_0*drone.k_t),
        (drone.k_d*std::sqrt(3))/(3*std::pow(drone.k_t,2)*drone.l_0),   -drone.k_d/(3*drone.l_0*std::pow(drone.k_t,2)),     -1/(3*drone.k_t),                                 -std::sqrt(3)/(3*drone.l_0*drone.k_t),    1/(3*drone.l_0*drone.k_t),  0,
        -(drone.k_d*std::sqrt(3))/(3*std::pow(drone.k_t,2)*drone.l_0),  -drone.k_d/(3*drone.l_0*std::pow(drone.k_t,2)),     -1/(3*drone.k_t),                                 std::sqrt(3)/(3*drone.l_0*drone.k_t),     1/(3*drone.l_0*drone.k_t),  0,
        0,                                                              (2*drone.k_d)/(3*drone.l_0*std::pow(drone.k_t,2)),  -1/(3*drone.k_t),                                 0,                                        -2/(3*drone.l_0*drone.k_t), 0).finished()),
    target_x(0),
    target_y(0),
    target_z(0),
    //target_yaw(0),
    x(0),
    y(0),
    z(0),
    yaw(0),
    w(0),
    qx(0),
    qy(0),
    qz(0) {}

motorData FlightController::calculate(double yawOffset) {
    imu.update_IMU();
    resetTargetAngle(target_q, target_x, target_y, target_z); //makes it target 0
    imu.getQuaternians(&q.w(), &q.x(), &q.y(), &q.z()); //get the current quaternion
    imu.getEulerRad(&roll, &pitch, &yaw); //get the current yaw
    imu.getLidarData(&z,&lidar2);//get the current angle and altitude
    imu.getAngularVelocity(&angular_velocity[0], &angular_velocity[1], &angular_velocity[2]); //get the current angular velocity
    z = 0.0;
    y = 0.0;
    x = 0.0;
    yaw -= yawOffset;

    x_error = (cos(yaw)*(target_x - x) + sin(yaw)*(target_y - y));
    U[0] = TransControlX.calculate(x_error);

    y_error = (-sin(yaw)*(target_x - x) + cos(yaw)*(target_y - y));
    U[1] = TransControlY.calculate(y_error);

    z_error = target_z - z;
    U[2] = TransControlZ.calculate(z_error) - drone.mass * drone.gravity;

    //calculate a error quaternion between q and target_q
    quad_error = (target_q * q.inverse());
  
    //calculate the control input for the tricopter orientation
    U.segment<3>(3) = pquad * (quad_error.coeffs().head(3) * (quad_error.w() > 0 ? 1 : -1)) + pquad2 * angular_velocity;
    
    Serial.print("U_roll: ");
    Serial.println(U[3]);

    
    //calculate the motor speeds
    Omega = M * U;

    Output.omega_1 = std::pow(Omega(0)*Omega(0) + Omega(3)*Omega(3), 0.25);
    Output.omega_2 = std::pow(Omega(1)*Omega(1) + Omega(4)*Omega(4), 0.25);
    Output.omega_3 = std::pow(Omega(2)*Omega(2) + Omega(5)*Omega(5), 0.25);

    Output.alpha_1 = std::atan(Omega(0)/Omega(3));
    Output.alpha_2 = std::atan(Omega(1)/Omega(4));
    Output.alpha_3 = std::atan(Omega(2)/Omega(5));

    return Output;
}
