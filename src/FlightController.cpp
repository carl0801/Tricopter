#include <Arduino.h>
#include "FlightController.h"

IMU imu;





void resetTargetAngle(double& yaw, double& x, double& y, double& z) {
    yaw = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

Tricopter::Tricopter(double mass, double l_0, double gravity, double drag, double j_x, double j_y, double j_z, double k_t, double k_d) :
    mass(mass), l_0(l_0), gravity(gravity), drag(drag), j_x(j_x), j_y(j_y), j_z(j_z), k_t(k_t), k_d(k_d) {
        l_1 = sin(60 * M_PI / 180) * l_0;
        l_2 = sin(-60 * M_PI / 180) * l_0;
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
    TransControlX(0.2, 0.01, 0.2, dt),
    TransControlY(0.2, 0.01, 0.2),
    TransControlZ(0.9545651916204778,0.04441356161831689,4.0843239598328),
    RotControlZ(0.08508958604791192, 0.00923672770996732, 0.4286804507510326),
    RotControlX(0.5, 0, 1.2),
    RotControlY(0.3, 0, 1.2, dt),
    drone(0.25, 0.25, 9.81, 0.02, 0.035, 0.035, 0.02, 0.0000008, 0.0000003),
    target_x(0),
    target_y(0),
    target_z(0),
    target_yaw(0),
    z(0),
    roll(0),
    pitch(0),
    yaw(0),
    omega_1(0),
    omega_2(0),
    omega_3(0),
    alpha(0) {}

motorData FlightController::calculate() {
    resetTargetAngle(target_yaw, target_x, target_y, target_z); //makes it target 0
    //time the get rotation function
    unsigned long start = millis();
    //imu.getQuaternionRotation(&q);
    imu.getEulerRotation(&roll, &pitch, &yaw); 
    unsigned long end = millis();
    Serial.print("Time to get rotation: "); Serial.println(end - start);
    start = millis();
    imu.getAltitude(&z);//get the current angle and altitude
    end = millis();
    Serial.print("Time to get altitude: "); Serial.println(end - start);

    //get the euler angles in radians from the quaternion using the Eigen library
    //Eigen::Matrix3f rot = q.normalized().toRotationMatrix();
    
    //convert rotationmatrix to XYZ euler angles
    //Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);

    //convert the target x from earth to drone frame
    

    


    z_error = target_z - z;
    U_Z = TransControlZ.calculate(z_error);
    U_Z -= drone.mass * drone.gravity;

    x_error = (target_x * cos(yaw) + target_y * sin(yaw)) - x;
    U_X = TransControlX.calculate(x_error);

    pitch_error = U_X - pitch;
    U_p = RotControlY.calculate(pitch_error);

    y_error = (target_y * cos(yaw) - target_x * sin(yaw)) - y;
    U_Y = TransControlY.calculate(y_error);

    roll_error = U_Y - roll;
    U_r = RotControlX.calculate(roll_error);

    yaw_error = target_yaw - yaw;
    U_y = RotControlZ.calculate(yaw_error);
    





    term1_12 = (2 * drone.l_0 * U_Z) / (drone.k_t * (drone.l_0 + drone.l_2));
    term2_12 = (2 * U_r) / (drone.l_1 * drone.k_t);
    term3_12 = (2 * U_p) / (drone.k_t * (drone.l_0 + drone.l_2));
    omega_1_mid = -term1_12 - term2_12 + term3_12; 
    omega_2_mid = -term1_12 + term2_12 + term3_12;
    Output.omega_1 = (omega_1_mid < 0) ? 0 : sqrt(omega_1_mid) / 2;
    Output.omega_2 = (omega_2_mid < 0) ? 0 : sqrt(omega_2_mid) / 2;

    term1_3p1 = ((drone.l_2 * U_Z) / (drone.k_t * (drone.l_0 + drone.l_2)));
    term1_3p2 = (U_p / (drone.k_t * (drone.l_0 + drone.l_2))) ;
    term1_3 = pow(-term1_3p1 - term1_3p2, 2);


    term2_3p1 = ((drone.k_d * U_Z) / (drone.k_t * drone.k_t * drone.l_0)) ;
    term2_3p2 = (U_y / (drone.l_0 * drone.k_t));
    term2_3 = pow(-term2_3p1 + term2_3p2, 2);

    Output.omega_3 = pow((term1_3 + term2_3), 0.25);



    Serial.print("omega1: "); Serial.println(omega_1);
    Serial.print("omega2: "); Serial.println(omega_2);
    Serial.print("omega3: "); Serial.println(omega_3);
    Serial.print("alpha: "); Serial.println(alpha*180/M_PI);


    alpha_term1 = -((drone.k_d * U_Z) / (drone.k_t * drone.k_t * drone.l_0) + U_y / (drone.l_0 * drone.k_t));
    alpha_term2 = -(drone.l_2 * U_Z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_p / (drone.k_t * (drone.l_0 + drone.l_2));
    Output.alpha = atan2(alpha_term1, alpha_term2);

    return Output;
}
