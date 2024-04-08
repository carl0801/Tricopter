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
    TransControlX(14.210199629344466, 2.743663460977311e-05, 0.6550344458063302, dt),
    TransControlY(-11.61166369003271, 3.1138720119081666, 21.95303938004841, dt),
    TransControlZ(4,0.0,1, dt),
    RotControlZ(0.08954740703540659, 0.009507839547302766, 0.411545168858215, dt),
    RotControlX(-0.0027784102834362123, 0.01739939755233525, 0.009438289952790207, dt),
    RotControlY(3.9177422768315404, 6.673890880922391, 0.9802682375214986, dt),
    drone(0.25, 0.25, 9.81, 0.02, 0.035, 0.035, 0.02, 0.0000008, 0.0000003),
    target_x(0),
    target_y(0),
    target_z(0),
    target_yaw(0),
    x(0),
    y(0),
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
    imu.getEulerRad(&roll_f, &pitch_f, &yaw_f);
    roll = static_cast<double>(roll_f);
    pitch = static_cast<double>(pitch_f);
    yaw = static_cast<double>(yaw_f);
    unsigned long end = millis();
    Serial.print("Time to get rotation: "); Serial.println(end - start);
    start = millis();
    imu.getLidarData(&z_f,&lidar2);//get the current angle and altitude
    z = static_cast<double>(z_f);
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

    x_error = (cos(yaw)*(target_x - x) + sin(yaw)*(target_y - y));
    U_X = TransControlX.calculate(x_error);

    pitch_error = U_X - pitch;
    U_p = RotControlY.calculate(pitch_error);

    y_error = (-sin(yaw)*(target_x - x) + cos(yaw)*(target_y - y));
    U_Y = TransControlY.calculate(y_error);

    roll_error = U_Y - roll;
    U_r = RotControlX.calculate(roll_error);

    yaw_error = target_yaw - yaw;
    U_y = RotControlZ.calculate(yaw_error);
    





    term1_12 = (drone.l_0 * U_Z) / (2*drone.k_t * (drone.l_0 + drone.l_2));
    term2_12 = (U_r) / (2*drone.l_1 * drone.k_t);
    term3_12 = (U_p) / (2*drone.k_t * (drone.l_0 + drone.l_2));
    omega_1_mid = -term1_12 - term2_12 + term3_12; 
    omega_2_mid = -term1_12 + term2_12 + term3_12;
    Output.omega_1 = (omega_1_mid < 0) ? 0 : sqrt(omega_1_mid) / 2;
    Output.omega_2 = (omega_2_mid < 0) ? 0 : sqrt(omega_2_mid) / 2;

    term1_3p1 = -((drone.l_2 * U_Z) / (drone.k_t * (drone.l_0 + drone.l_2)));
    term1_3p2 = -(U_p / (drone.k_t * (drone.l_0 + drone.l_2))) ;
    term1_3 = pow(-term1_3p1 - term1_3p2, 2);


    term2_3p1 = -((drone.k_d * U_Z) / (drone.k_t * drone.k_t * drone.l_0)) ;
    term2_3p2 = (U_y / (drone.l_0 * drone.k_t));
    term2_3 = pow(-term2_3p1 + term2_3p2, 2);

    Output.omega_3 = pow((term1_3 + term2_3), 0.25);



    Serial.print("omega1: "); Serial.println(omega_1);
    Serial.print("omega2: "); Serial.println(omega_2);
    Serial.print("omega3: "); Serial.println(omega_3);
    Serial.print("alpha: "); Serial.println(alpha*180/M_PI);


    alpha_term1 = -((drone.k_d * U_Z) / (drone.k_t * drone.k_t * drone.l_0) + U_y / (drone.l_0 * drone.k_t));
    alpha_term2 = (drone.l_2 * U_Z) / (drone.k_t * (drone.l_0 + drone.l_2)) - U_p / (drone.k_t * (drone.l_0 + drone.l_2));
    Output.alpha = atan(alpha_term1 / alpha_term2);

    return Output;
}
