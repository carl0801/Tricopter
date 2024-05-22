#include <Arduino.h>
#include "FlightController.h"


void resetTargetAngle(Eigen::Quaterniond& q, double& x, double& y, double& z) {
    q = Eigen::Quaterniond(1, 0, 0, 0);
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

Eigen::Vector3d quaternionToVector(const Eigen::Quaterniond& q) {
    Eigen::Vector3d q_v = q.vec();
    double q_r = q.w();
    double norm_q_v = q_v.norm();
    double angle = 2 * atan2(norm_q_v, q_r);
    return (norm_q_v > 0) ? (q_v / norm_q_v) * angle : Eigen::Vector3d(0, 0, 0);
}

Tricopter::Tricopter(double mass, double l_0, double gravity, double drag, double j_x, double j_y, double j_z, double k_t1, double k_t2, double k_t3, double k_d) :
    mass(mass), l_0(l_0), gravity(gravity), drag(drag), j_x(j_x), j_y(j_y), j_z(j_z), k_t1(k_t1), k_t2(k_t2), k_t3(k_t3), k_d(k_d) {
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
    TransControlX(6,0,0, dt),
    TransControlY(6,0,0, dt),
    TransControlZ(2,0,0, dt),        //0.025, 0.035, 0.05                -- 0.005, 0.005, 0.005                -- 0.025, 0.03, 0.03
    OrientationControl(Eigen::Vector3d(0.035, 0.035, 0.05), Eigen::Vector3d(0.0005, 0.0005, 0.005), Eigen::Vector3d(0.02, 0.02, 0.03), dt, Eigen::Vector3d(10, 10, 10), 0.1),
    //OrientationControl(0.05, 0.005, 0.03, dt, 10, 0.1),
    //pquad is a 3x3 diagonal matrix with 0.1 in the first diagonal slot, 10 in the middle and 1 in the last
    pquad((Eigen::Vector3d(1.5, 1.5, 0.25)).asDiagonal()), //0.5, 0.5, 0.2
    //iden is a 3x3 identity matrix * 0.01
    pquad2((Eigen::Vector3d(5, 5, 5)).asDiagonal()), //0.3, 0.3, 1
    
    drone(0.5, 0.33, 9.81, 0.02, 0.035, 0.035, 0.02, 5.369e-7, 5.115e-7, 5.485e-7, 5.295e-8), //6.769e-6 7.295e-8 hh 5.8922e-6,5.4468e-6,4.914e-6, 7.295e-8 hh 6.170e-7,5.704e-7,5.146e-7, 7.295e-8
    //  1                                                          2                                                    3                                                 4                                         5                                6        
    /* M((Eigen::Matrix<double, 6, 6>() << 
        -std::sqrt(3)/(3*drone.k_t1),                                    1/(3*drone.k_t2),                                    drone.k_d/(3*drone.l_0*std::pow(drone.k_t3,2)),   0,                                        0,                            1/(3*drone.l_0*drone.k_t3),
        std::sqrt(3)/(3*drone.k_t1),                                     1/(3*drone.k_t2),                                    drone.k_d/(3*drone.l_0*std::pow(drone.k_t3,2)),   0,                                        0,                            1/(3*drone.l_0*drone.k_t3),
        0,                                                              -2/(3*drone.k_t2),                                    drone.k_d/(3*drone.l_0*std::pow(drone.k_t3,2)),   0,                                        0,                            1/(3*drone.l_0*drone.k_t3),
        (-drone.k_d*std::sqrt(3))/(3*std::pow(drone.k_t1,2)*drone.l_0),   drone.k_d/(3*drone.l_0*std::pow(drone.k_t2,2)),     -1/(3*drone.k_t3),                                 -std::sqrt(3)/(3*drone.l_0*drone.k_t1),    1/(3*drone.l_0*drone.k_t2),  0,
        -(-drone.k_d*std::sqrt(3))/(3*std::pow(drone.k_t1,2)*drone.l_0),  drone.k_d/(3*drone.l_0*std::pow(drone.k_t2,2)),     -1/(3*drone.k_t3),                                 std::sqrt(3)/(3*drone.l_0*drone.k_t1),     1/(3*drone.l_0*drone.k_t2),  0,
        0,                                                              (2*-drone.k_d)/(3*drone.l_0*std::pow(drone.k_t2,2)),  -1/(3*drone.k_t3),                                 0,                                        -2/(3*drone.l_0*drone.k_t2),   0).finished()), */
    //  1                                           2                              3                               4                          5                           6        
    M((Eigen::Matrix<double, 6, 6>() << 
    -620851.0320*sqrt(3),                      620842.8282,                  -10435.33737,                517.8940403*sqrt(3),               951.5961675,                 0.1881357355*pow(10,7),
     651674.9707*sqrt(3),                      651672.5599,                  -10953.53399,                543.6115548*sqrt(3),               998.8504053,                 0.1974781552*pow(10,7),
    -2.891080073*sqrt(3),                     -0.1215441177*pow(10,7),       -10214.64473,                506.9413133*sqrt(3),               931.4712531,                 0.1841569306*pow(10,7),
     10508.23701*sqrt(3),                     -10020.28123,                  -620849.7297,               -0.1881357731*pow(10,7)*sqrt(3),    0.1881357964*pow(10,7),      297.6809031,
    -11209.65550*sqrt(3),                     -11056.67274,                  -651670.7478,                0.1974781670*pow(10,7)*sqrt(3),    0.1974781365*pow(10,7),     -1320.294849,
     167.4864884*sqrt(3),                      20119.19253,                  -607723.2318,                0.2593977010*sqrt(3),             -0.3683139032*pow(10,7),      939.8467427).finished()),

        
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
    qz(0)
    {};


motorData FlightController::calculate(double yawOffset,Eigen::Quaterniond target_q, double target_x, double target_y, double target_z) {
    imu.update_IMU();
    resetTargetAngle(target_q, target_x, target_y, target_z); //makes it target 0
    imu.getQuaternians(&q.w(), &q.x(), &q.y(), &q.z()); //get the current quaternion
    imu.getPos(&x, &y, &z); //get the current position
    imu.getEulerRad(&roll, &pitch, &yaw); //get the current yaw
    imu.getYaw(&yaw);
    imu.getLidarData(&z,&lidar2);//get the current angle and altitude
    imu.getAngularVelocity(&angular_velocity[0], &angular_velocity[1], &angular_velocity[2]); //get the current angular velocity
    x = 0.0;
    y = 0.0;
    z = 0.0;



    yaw -= yawOffset;



    x_error = (cos(yaw)*(target_x - x) + sin(yaw)*(target_y - y));
    U[0] = TransControlX.calculate(x_error);

    y_error = (-sin(yaw)*(target_x - x) + cos(yaw)*(target_y - y));
    U[1] = TransControlY.calculate(y_error);

    z_error = target_z - z;
    U[2] = TransControlZ.calculate(z_error) - drone.mass * drone.gravity;

    //calculate a error quaternion between q and target_q
    quad_error = target_q * q.inverse();

    error_vector = Eigen::Vector3d(quad_error.x(), quad_error.y(), quad_error.z());
    //U.segment<3>(3) = pquad * error_vector * (quad_error.w() > 0 ? 1 : -1)  + pquad2 * angular_velocity;
    U.segment<3>(3) = -pquad * error_vector * (quad_error.w() > 0 ? 1 : -1)  - pquad2 * angular_velocity;
    // Convert quaternion error to vector
    /* Eigen::Vector3d error_vector = quaternionToVector(quad_error);
    //switch x and y
    error_vector = Eigen::Vector3d(error_vector[0], error_vector[1], error_vector[2]);

    //error_vector = error_vector * (quad_error.w() > 0 ? 1 : -1);
    // Calculate the control input for the tricopter orientation using the PID controller
    Eigen::Vector3d pid_output = OrientationControl.calculate(error_vector);

    //U.segment<3>(3) = pquad * pid_output + pquad2 * angular_velocity;
    
    
    U.segment<3>(3) = pid_output;
    U.segment<3>(3) = Eigen::Vector3d(U[3], U[4], U[5]); */


    Serial.print("U_roll: ");
    Serial.println(U[3]);
    Serial.print("U_pitch: ");
    Serial.println(U[4]);
    Serial.print("U_yaw: ");
    Serial.println(U[5]);


    //calculate the control input for the tricopter orientation
    //U.segment<3>(3) = pquad * (quad_error.coeffs().head(3) * (quad_error.w() > 0 ? 1 : -1)) + pquad2 * angular_velocity;
    
    //calculate the motor speeds
    Omega = M * U;

    Output.omega_1 = std::sqrt(Omega(0)*Omega(0) + Omega(3)*Omega(3));
    Output.omega_2 = std::sqrt(Omega(1)*Omega(1) + Omega(4)*Omega(4));
    Output.omega_3 = std::sqrt(Omega(2)*Omega(2) + Omega(5)*Omega(5));


    Output.alpha_1 = std::atan(Omega(0)/Omega(3));
    Output.alpha_2 = std::atan(Omega(1)/Omega(4));
    Output.alpha_3 = std::atan(Omega(2)/Omega(5));


    return Output;
}




