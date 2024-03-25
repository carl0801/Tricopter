#include <Arduino.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "MPU6050.h"
#include "HMC5883L.h"
#include <Adafruit_BMP085.h>
#include "IMU.h"
#include "Fusion.h"


/*------------------------------- VARIBLES ----------------------------*/

const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
const double ALPHA = 0.8; // alpha value for the complementary filter
double calibrated_values[3];

struct RawData{
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
};
RawData raw;

struct Mag{
  double x = 0;
  double y = 0;
  double z = 0;
  double calX = 0; double calY = 0; double calZ = 0;
};  
Mag mag;

struct filterData{
  float gx, gy, gz;
  float ax, ay, az;
  float mx, my, mz;
};
filterData data;




/*------------------------------- CLASSES -----------------------------*/

// class for the MPU6050
MPU6050 accelgyro;

// class for the HMC5883L
HMC5883L magnometer;

// class for BMP085
Adafruit_BMP085 bmp;

// class for vl53l0x
VL53L0X lidar;

// class for the AHRS algorithm
FusionAhrs ahrs;

/*------------------------------- FUNCTIONS ----------------------------*/

// Initialize the sensors
void IMU::init_sensors() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize MPU6050
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true);
  accelgyro.setSleepEnabled(false);

  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(0);
  accelgyro.setFullScaleAccelRange(0);
  accelgyro.CalibrateAccel();
  accelgyro.CalibrateGyro();

  // initialize the magnometer
  magnometer.initialize();

  // initialize the BMP180
  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

  // initialize the ToF sensor
  lidar.setTimeout(500);
  if (!lidar.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  lidar.startContinuous();


  // Initialise algorithms
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .convention = FusionConventionNwu,
          .gain = 5.0f,
          .gyroscopeRange = 250.0f, /* replace this with actual gyroscope range in degrees/s */
          .accelerationRejection = 90.0f,
          .magneticRejection = 90.0f,
  };
  FusionAhrsSetSettings(&ahrs, &settings);
  

}

// Compute the calibration mag values
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

// Get the current reading from the sensors
filterData readSensor(filterData data){
    // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&raw.ax, &raw.ay, &raw.az, &raw.gx, &raw.gy, &raw.gz);

  // read raw magnetometer measurements from device
  magnometer.getHeading(&raw.mx, &raw.my, &raw.mz);

  // Apply the calibration matrix to the magnetometer
  mag = transformation(raw, mag);

  data.gx = raw.gx / 131;
  data.gy = raw.gy / 131;
  data.gz = raw.gz / 131;

  data.ax = raw.ax / 16384.0;
  data.ay = raw.ay / 16384.0;
  data.az = raw.az / 16384.0;

  data.mx = mag.calX;
  data.my = mag.calY;
  data.mz = mag.calZ;

  return data;
}

//Get euler rotation
void IMU::getEulerRotation(double *roll, double *pitch, double *yaw) {

  data = readSensor(data);

  FusionVector gyroscope = {data.gx, data.gy, data.gz}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {data.ax, data.ay, data.az}; // replace this with actual accelerometer data in g
  FusionVector magnetometer = {data.mx, data.my, data.mz}; // replace this with actual magnetometer data in arbitrary units

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, SAMPLE_PERIOD);

  const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
  const FusionEuler euler = FusionQuaternionToEuler(quaternion);

  *roll = euler.angle.roll;
  *pitch = euler.angle.pitch;
  *yaw = euler.angle.yaw;

}

//Get quaternion rotation
//void IMU::getQuaternionRotation(double *w, double *x, double *y, double *z) {
void IMU::getQuaternionRotation(Eigen::Quaterniond *q) {
  data = readSensor(data);

  FusionVector gyroscope = {data.gx, data.gy, data.gz}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {data.ax, data.ay, data.az}; // replace this with actual accelerometer data in g
  FusionVector magnetometer = {data.mx, data.my, data.mz}; // replace this with actual magnetometer data in arbitrary units

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, SAMPLE_PERIOD);

  const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);
  const FusionEuler euler = FusionQuaternionToEuler(quaternion);

  *q = Eigen::Quaterniond(quaternion.element.w, quaternion.element.x, quaternion.element.y, quaternion.element.z);
}

//Get alttitude from ToF sensor
void IMU::getAltitude(double *altitude) {
  *altitude = lidar.readRangeContinuousMillimeters();
};

// Get the pressure from the BMP085 (Pa)
void IMU::getPressure(double *pressure) {
  *pressure = bmp.readPressure();
}

// Get the temperature from the BMP085 (C°)
void IMU::getTemperature(double *temperature) {
  *temperature = bmp.readTemperature();
}

// Function to test the sensors
void IMU::test(float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float *mx, float *my, float *mz){
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&raw.ax, &raw.ay, &raw.az, &raw.gx, &raw.gy, &raw.gz);

  // read raw magnetometer measurements from device
  magnometer.getHeading(&raw.mx, &raw.my, &raw.mz);

  // Apply the calibration matrix to the magnetometer
  mag = transformation(raw, mag);
  
  // apply the complementary filter
  //angle = complementaryFilter(angle, raw, mag);

  data.gx = raw.gx / 32.8;
  data.gy = raw.gy / 32.8;
  data.gz = raw.gz / 32.8;

  data.ax = raw.ax / 16384.0;
  data.ay = raw.ay / 16384.0;
  data.az = raw.az / 16384.0;

  data.mx = mag.calX;
  data.my = mag.calY;
  data.mz = mag.calZ;
  
  *gx = data.gx;
  *gy = data.gy;
  *gz = data.gz;
  *ax = data.ax;
  *ay = data.ay;
  *az = data.az;
  *mx = data.mx;
  *my = data.my;
  *mz = data.mz;

}
