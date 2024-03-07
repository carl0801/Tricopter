#include <Arduino.h>

#include "I2Cdev.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include "IMU.h"


/*------------------------------- VARIBLES ----------------------------*/

const double STEP_TIME = 20; // timeInterval
const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
const double ALPHA = 0.8; // alpha value for the complementary filter
double calibrated_values[3];

struct RawData{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
};
RawData raw;

struct Angle{
  double z1 = 0;
  double z2 = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
};
Angle angle;

struct Mag{
  double x = 0;
  double y = 0;
  double z = 0;
  double calX = 0; double calY = 0; double calZ = 0;
};  
Mag mag;


/*------------------------------- CLASSES -----------------------------*/

// class for the MPU6050 and HMC5883L
MPU6050 accelgyro;
HMC5883L magnometer;

// class for vl53l0x
VL53L0X lidar;

/*------------------------------- FUNCTIONS ----------------------------*/

// Function to initialize the sensors
void IMU::init_sensors() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);

  // initialize MPU6050
  Serial.println("Initializing I2C devices...");
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true);
  accelgyro.setSleepEnabled(false);

  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(2);
  accelgyro.setFullScaleAccelRange(2);
  accelgyro.setXGyroOffset(58);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(-647);
  accelgyro.CalibrateAccel();

  // initialize the magnometer
  magnometer.initialize();


  // initialize the ToF sensor
  lidar.setTimeout(500);
  if (!lidar.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  lidar.startContinuous();

}

// Function to compute the dot product of a Vector3D and a 3x3 matrix
Mag transformation(RawData raw, Mag mag)    
{
  mag.x = raw.mx;
  mag.y = raw.my;
  mag.z = raw.mz;
  double uncalibrated_values[3] = {mag.x, mag.y, mag.z};

  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] = {
      {1.186, 0.002, 0.038},
      {0.046, 1.209, -0.028},
      {-0.006, 0.05, 1.231}
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] = 
  {
    155.373,
    -245.196,
    96.855
  };  
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];

  mag.calX = calibrated_values[0];
  mag.calY = calibrated_values[1];
  mag.calZ = calibrated_values[2];
  return mag;
}

// Function to apply the complementary filter
Angle complementaryFilter(Angle angle, RawData raw, Mag mag) {
  double currentAngle[3]{angle.roll, angle.pitch, angle.yaw};
  double newAngle[3];
  double gyroAngleChange[3];

  // Apply the calibration matrix to the magnetometer
  mag = transformation(raw, mag);

  // Calculate the change in angle from the gyro
  gyroAngleChange[0] = ((raw.gx) * (STEP_TIME / 1000)) / 32.8; // 32.8 is the sensitivity of the gyro
  gyroAngleChange[1] = ((raw.gy) * (STEP_TIME / 1000)) / 32.8; // 32.8 is the sensitivity of the gyro
  gyroAngleChange[2] = ((raw.gz) * (STEP_TIME / 1000)) / 32.8; // 32.8 is the sensitivity of the gyro


  // Calculate the angle from the accelerometer and magnetometer
  newAngle[0] = atan2(raw.ay / 16384.0, sqrt(((raw.ax/ 16384.0) * (raw.ax/ 16384.0) + (raw.az/ 16384.0) * (raw.az/16384.0)))); // 16384 is the sensitivity of the accelerometer
  newAngle[1] = atan2(raw.ax / 16384.0, sqrt(((raw.ay/ 16384.0) * (raw.ay/ 16384.0) + (raw.az/ 16384.0) * (raw.az/ 16384.0))));


  // Tilt compensation
  double tiltCompensatedY = -mag.calY * cos(newAngle[0]) + mag.calZ * sin(newAngle[0]);
  double tiltCompensatedX = mag.calX * cos(newAngle[1]) - mag.calY * sin(newAngle[1]) * sin(newAngle[0]) - mag.calZ * sin(newAngle[1]) * cos(newAngle[0]);
  
  newAngle[2] = atan2(tiltCompensatedY, tiltCompensatedX);

  for (int i=0; i < 3; ++i) {
    
    // Convert to degrees
    newAngle[i] = newAngle[i] * 180/PI;

    // Apply the complementary filter
    currentAngle[i] = (ALPHA * (currentAngle[i] + gyroAngleChange[i])) + ((1 - ALPHA) * newAngle[i]); 
    
  }

  angle.roll = currentAngle[0];
  angle.pitch = currentAngle[1];
  angle.yaw = currentAngle[2];

  return angle;
}

//Get roll, pitch and yaw angle and alttitude from ToF sensor
void IMU::getIMUData(double *roll, double *pitch, double *yaw, double *z1) {

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&raw.ax, &raw.ay, &raw.az, &raw.gx, &raw.gy, &raw.gz);

  // read raw magnetometer measurements from device
  magnometer.getHeading(&raw.mx, &raw.my, &raw.mz);

  // apply the complementary filter
  angle = complementaryFilter(angle, raw, mag);

  angle.z1 = lidar.readRangeContinuousMillimeters();

  *roll = angle.roll;
  *pitch = angle.pitch;
  *yaw = angle.yaw;
  *z1 = angle.z1;

}

