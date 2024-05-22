#include <WiFi.h>
#include <ESP32Servo.h>
#include "freertos/task.h"
#include <Wire.h>
#include <Arduino.h>

class Emil {
private:
    IMU& imu;
    motorData& motorValues;
double integral_x = 0.0;
double integral_y = 0.0;
double integral_alti = 0.00;//  # Integral term for altitude control
double previous_error_x = 0.0;
double previous_error_y = 0.0;
double previous_error_alti = 0.0; // # Previous error for altitude control

double target_roll = 0.0;
double target_pitch = 0.0;
double target_altitude = -700.0; //0.5
double target_yaw = 0.0;
double output1 = 0.0;
double output2 = 0.0;
double output3 = 0.0;

double current_roll;
double current_pitch;
double current_yaw;
double target_x=515.0;
double target_y=244.0;
double error_orientation_x = 0.0;
double error_orientation_y = 0.0;
double error_altitude = 0.0;
double derivative_x = 0.0;
double derivative_y = 0.0;
double derivative_alti = 0.0;
double time_step = 0.005;
double altitude = 0.0;
double timeofFlight = 0.0;
double offset_pitch = 1.0;
double U_r1 = 0.0;
double U_r2 = 0.0;
double U_p1 = 0.0;
double U_p2 = 0.0;
double U_p3 = 0.0;
double U_alti = 0.0;
double map_scale = 2;
// PID Parameters (from simulation)
double kp_2 = 0.3;    // Roll and pitch proportional gain
double ki = 0.1;      // Roll and pitch integral gain   0.1
double kd = 0.15;     // Roll and pitch derivative gain
double kp = 1.5;      // Altitude proportional gain
double ki_alti = 0.0; // Altitude integral gain    0.3
double kd_alti = 1.5; // Altitude derivative gain

// Yaw Control Parameters (from simulation)
double kp_yaw = 0.2;
double ki_yaw = 0.0; //0.1
double kd_yaw = 0.15;

// Additional Variables for Position Control (from simulation)
double integral_position_x = 0.0;
double integral_position_y = 0.0;
double integral_yaw = 0.0;
double previous_error_position_x = 0.0;
double previous_error_position_y = 0.0;
double previous_error_yaw = 0.0;
double kp_position_x = 0.00001;
double kp_position_y = 0.00001;
double tilt_angle = 0.0;


public:
Emil(IMU& imuRef, motorData& motorValuesRef) 
    : imu(imuRef), motorValues(motorValuesRef) {
    // Constructor logic here
}

//a function that turns a quaternion into euler angles
void quat2eul(double q[4], double euler[3]){
  double sinr_cosp = 2*(q[0]*q[1] + q[2]*q[3]);
  double cosr_cosp = 1 - 2*(q[1]*q[1] + q[2]*q[2]);
  euler[0] = atan2(sinr_cosp, cosr_cosp);

  double sinp = 2*(q[0]*q[2] - q[3]*q[1]);
  if (abs(sinp) >= 1)
    euler[1] = copysign(M_PI/2, sinp);
  else
    euler[1] = asin(sinp);

  double siny_cosp = 2*(q[0]*q[3] + q[1]*q[2]);
  double cosy_cosp = 1 - 2*(q[2]*q[2] + q[3]*q[3]);
  euler[2] = atan2(siny_cosp, cosy_cosp);
}

void control(double pos[3],double pwm_offset = 20){
  imu.update_IMU();
  imu.getEulerRad(&current_roll, &current_pitch, &current_yaw);
  current_roll *= M_PI/180;
  current_pitch *= M_PI/180;
  current_yaw *= M_PI/180;

  //imu.getLidarData(&altitude, &timeofFlight);

  // Position Control (Added from simulation)
  double error_x = target_x - pos[0];                 // Tilføj positions data her hvis i har det. istedet for 0
  double error_y = -(target_y - pos[1]);
  error_x = 0;
  error_y = 0;

  //pos[2] *= -1;


  integral_position_x += error_x * time_step;
  integral_position_y += error_y * time_step;

  double derivative_position_x = (error_x - previous_error_position_x) / time_step;
  double derivative_position_y = (error_y - previous_error_position_y) / time_step;

  double pitch_error = kp_position_x * error_x; // + ki_position_x * integral_position_x + kd_position_x * derivative_position_x;
  double roll_error = kp_position_y * error_y;  // + ki_position_y * integral_position_y + kd_position_y * derivative_position_y;

  // Adjust Tilt Angles Based on Position Errors
  double additional_tilt_m1 = pitch_error + roll_error * 0.5;
  double additional_tilt_m2 = -pitch_error - roll_error * 0.5;
  double additional_tilt_m3 = 0 - roll_error;

  // Yaw Control (Added from simulation)
  double yaw_error = -(target_yaw - current_yaw); // Assuming yaw is in radians
  integral_yaw += yaw_error * time_step;
  double derivative_yaw = (yaw_error - previous_error_yaw) / time_step;
  tilt_angle = kp_yaw * yaw_error + ki_yaw * integral_yaw + kd_yaw * derivative_yaw;

  // Apply Tilt Adjustments to Orientation Errors
  error_orientation_x = target_roll - current_roll + additional_tilt_m1; 
  error_orientation_y = target_pitch - current_pitch + additional_tilt_m2; 
  error_altitude = target_altitude - altitude + additional_tilt_m3; 
  // Set target positions
  /* target_altitude = z;
  target_x = x;
  target_y = y; */

  error_orientation_x = RAD_TO_DEG*(target_roll -current_roll);
  error_orientation_y = RAD_TO_DEG*(target_pitch - current_pitch);
  error_altitude = target_altitude - pos[2];  // Altitude error;

  // Update integral terms
  integral_x += error_orientation_x * time_step;
  integral_y += error_orientation_y * time_step;
  integral_alti += error_altitude * time_step;  // Altitude integral term

  //  # Update derivative terms
  derivative_x = (error_orientation_x - previous_error_x) / time_step;
  derivative_y = (error_orientation_y - previous_error_y) / time_step;
  derivative_alti = (error_altitude - previous_error_alti) / time_step;//  # Altitude derivative term

  // Calculate control inputs
  
  U_alti = kp * error_altitude + ki_alti * integral_alti + kd_alti * derivative_alti;//  # Altitude control

  U_r1 = kp_2 * error_orientation_x + ki * integral_x + kd * derivative_x;
  U_r2 = -kp_2 * error_orientation_x + (-ki * integral_x) + (-kd * derivative_x);
  U_p1 = (-kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y))*-1;
  U_p2 = (-kp_2 * error_orientation_y + (-ki * integral_y) + (-kd * derivative_y))*-1;
  U_p3 = (kp_2 * error_orientation_y  + ki * integral_y + kd * derivative_y)*-1;
r
  // Combine control inputs
 
  output3 = map_scale*(offset_pitch +U_alti + U_r1 + U_p1);
  output1 = map_scale*(offset_pitch +U_alti + U_r2 + U_p2);
  output2 = map_scale*(offset_pitch +U_alti + U_p3);  

  constrain(output1, 0, 120);
  constrain(output2, 0, 120);
  constrain(output3, 0, 120);

  tilt_angle *= -1;
  tilt_angle = constrain(tilt_angle+90,30,150);
  


  motorValues.omega_1 = output1;
  motorValues.omega_2 = output3;
  motorValues.omega_3 = output2;

  motorValues.omega_1 += pwm_offset;
  motorValues.omega_2 += pwm_offset;
  motorValues.omega_3 += pwm_offset;


  motorValues.alpha_1 = tilt_angle;
  motorValues.alpha_2 = tilt_angle;
  motorValues.alpha_3 = tilt_angle;

  Serial.print(" x: ");
  Serial.print(pos[0]);
  Serial.print(" y: ");
  Serial.print(pos[1]);
  Serial.print(" z: ");
  Serial.println(pos[2]);

  /* Serial.print("omega_1: ");
  Serial.print(motorValues.omega_1);
  Serial.print(" omega_2: ");
  Serial.print(motorValues.omega_2);
  Serial.print(" omega_3: ");
  Serial.print(motorValues.omega_3);
  Serial.print(" alpha: ");
  Serial.println(tilt_angle); */

  /* Serial.print("x: ");
  Serial.print(pos[0]);
  Serial.print(" y: ");
  Serial.print(pos[1]);
  Serial.print(" z: ");
  Serial.println(pos[2]); */



  /* updateESC1(output1);
  updateESC2(output2);
  updateESC3(output3);
  alpha1.write(tilt_angle);
  alpha2.write(tilt_angle);
  alpha3.write(tilt_angle); */
  

  previous_error_x = error_orientation_x;
  previous_error_y = error_orientation_y;
  previous_error_alti = error_altitude;

  //return motorValues;
  
}
};



