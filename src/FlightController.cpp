#include <Arduino.h>
#include "FlightController.h"

IMU imu;

Servo esc1;
Servo esc2;
Servo esc3;

void updateMotor(double omega_1, double omega_2, double omega_3, double alpha) {
//map the values to the correct of range 0-180 from 0-2500
  omega_1 = map(omega_1, 0, 4000, 0, 100);
  omega_2 = map(omega_2, 0, 4000, 0, 100);
  omega_3 = map(omega_3, 0, 4000, 0, 100);
  
  //make sure the value is max 100
  omega_1 = std::min(omega_1, 100.0);
  omega_2 = std::min(omega_2, 100.0);
  omega_3 = std::min(omega_3, 100.0);

  esc1.write(omega_1);
  esc2.write(omega_2);
  esc3.write(omega_3);
}

void resetTargetAngle(double& roll, double& pitch, double& yaw, double& z) {
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
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
    TransControlZ(0.5,0,0,dt),
    RotControlZ(0.21,0,0,dt),
    RotControlX(1.5,0,0,dt),
    RotControlY(1.5,0,0,dt),
    drone(0.25, 0.25, 9.81, 0.02, 0.035, 0.035, 0.02, 0.00002, 0.000003),
    target_z(0),
    target_roll(0),
    target_pitch(0),
    target_yaw(0),
    z(0),
    roll(0),
    pitch(0),
    yaw(0),
    omega_1(0),
    omega_2(0),
    omega_3(0),
    alpha(0) {}

std::tuple<double, double, double, double> FlightController::calculate() {
        resetTargetAngle(target_roll, target_pitch, target_yaw, target_z);
        imu.getRotation(&roll, &pitch, &yaw);  //get the current angle
        imu.getAltitude(&z); //get the current altitude

        //z = z;
        roll *= M_PI/180;
        pitch *= M_PI/180;
        yaw *= M_PI/180;

        
        double z_error = target_z - z;
        double U_z = TransControlZ.calculate(-z_error);
        double roll_error = target_roll - roll;
        double U_r = RotControlX.calculate(-roll_error);
        double pitch_error = target_pitch - pitch;
        double U_p = RotControlY.calculate(pitch_error);
        double yaw_error = target_yaw - yaw;
        double U_y = RotControlZ.calculate(yaw_error);
        

        U_z = U_z - drone.gravity*drone.mass;

 


        double term1_12 = (2 * drone.l_0 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2));
        double term2_12 = (2 * U_r) / (drone.l_1 * drone.k_t);
        double term3_12 = (2 * U_p) / (drone.k_t * (drone.l_0 + drone.l_2));
        double omega_1_mid = -term1_12 - term2_12 + term3_12; 
        double omega_2_mid = -term1_12 + term2_12 + term3_12;
        omega_1 = (omega_1_mid < 0) ? 0 : sqrt(omega_1_mid) / 2;
        omega_2 = (omega_2_mid < 0) ? 0 : sqrt(omega_2_mid) / 2;

 

        long double term1_3p1 = ((drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)));
        long double term1_3p2 = (U_p / (drone.k_t * (drone.l_0 + drone.l_2))) ;
        long double term1_3 = pow(term1_3p1 - term1_3p2, 2);


        long double term2_3p1 = ((drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0)) ;
        long double term2_3p2 = (U_y / (drone.l_0 * drone.k_t));

        long double term2_3 = pow(term2_3p1 + term2_3p2, 2);
 
        omega_3 = pow((term1_3 + term2_3), 0.25);



        Serial.print("omega1: "); Serial.println(omega_1);
        Serial.print("omega2: "); Serial.println(omega_2);
        Serial.print("omega3: "); Serial.println(omega_3);
        Serial.print("alpha: "); Serial.println(alpha*180/M_PI);


        double alpha_term1 = -((drone.k_d * U_z) / (drone.k_t * drone.k_t * drone.l_0) + U_y / (drone.l_0 * drone.k_t));
        double alpha_term2 = (drone.l_2 * U_z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_p / (drone.k_t * (drone.l_0 + drone.l_2));
        alpha = atan2(alpha_term1, alpha_term2);

    updateMotor(omega_1, omega_2, omega_3, alpha);
    // Return results
    return std::make_tuple(omega_1, omega_2, omega_3, alpha);
}
