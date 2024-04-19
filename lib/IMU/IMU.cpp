#include "IMU.h"

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.03f, 0.0f, -0.005f};
const FusionMatrix softIronMatrix = {0.950194, -0.023079, 0.0223308, -0.023079, 0.977227, -0.0143745, 0.0223308, -0.0143745, 0.939388}; //{0.970484, -0.0013793, 0.0404112, -0.0013793, 0.98968, 0.0134713, 0.0404112, 0.0134713, 0.93159}; 
const FusionVector hardIronOffset = {-94.6133, -5.1279, 20.4048};//{-113.612, 0.10103, 24.6554};

FusionVector gyroscope = {0,0,0};  
FusionVector accelerometer = {0,0,0}; 
FusionVector magnetometer = {0,0,0};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

MPU9250 MPU(Wire, MPU9250_ADDRESS);


#ifdef VL53L0X_CONNECT
  // The number of sensors in your system.
  const uint8_t sensorCount = 1;

  // The Arduino pin connected to the XSHUT pin of each sensor.
  const uint8_t xshutPins[sensorCount] = { 4 };

  VL53L0X sensors[sensorCount];

#endif //VL53L0X_ADDRESS

IMU::IMU(double dt):
//static_cast it to a unsigned int
  SAMPLE_RATE(static_cast<unsigned int>(1/(dt/1000)))
{}

// Read data from MPU9250
void IMU::read_sensors() {
  MPU.readSensor();
  accel[0] = MPU.getAccelX_mss();
  accel[1] = MPU.getAccelY_mss();
  accel[2] = MPU.getAccelZ_mss();

  magnetom[0] = MPU.getMagX_uT();
  magnetom[1] = MPU.getMagY_uT();
  magnetom[2] = MPU.getMagZ_uT();

  gyro[0] = MPU.getGyroX_rads();
  gyro[1] = MPU.getGyroY_rads();
  gyro[2] = MPU.getGyroZ_rads();
}

// Send data to PC
void IMU::sendToPC(double* data1, double* data2, double* data3){ 

  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[20] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 20);
}

// Update the IMU
void IMU::update_IMU() {

  //Read sensor data from MPU9250
  read_sensors();

  // Calculate time since last loop
  time_now = micros();
  deltat = (double)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  
  gyroscope = {gyro[0], gyro[1], gyro[2]};  
  accelerometer = {accel[0], accel[1], accel[2]}; 
  magnetometer = {magnetom[0], magnetom[1], magnetom[2]};

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // Update gyroscope AHRS algorithm
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltat);

  // Print algorithm outputs
  const FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

  // Get Euler angles (Z-Y-X convention)
  const FusionEuler euler = FusionQuaternionToEuler(quaternion);

  // Get Earth acceleration
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  // Get gyroscope data
  const FusionVector halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5f));

  // Angular velocity
  const FusionVector angvel = FusionVectorMultiplyScalar(FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(FusionVectorAdd(ahrs.halfAccelerometerFeedback, ahrs.halfMagnetometerFeedback), ahrs.rampedGain)), deltat);

  // Update acceleration
  acceleration[0] = (earth.axis.x * GRAVITY + 0.009331);
  acceleration[1] = (earth.axis.y * GRAVITY + 0.008373);
  acceleration[2] = -(earth.axis.z * GRAVITY + 0.419104);

  // Update velocity
  for (int i = 0; i < 3; i++) {
    if (acceleration[i] < 0.01 && acceleration[i] > -0.01) {
      velocity[i] = 0.0f;
    }
    else {
      velocity[i] += acceleration[i] * deltat;
    }
  }

  // Update position
  position[0] += velocity[0] * deltat;
  position[1] += velocity[1] * deltat;
  position[2] += velocity[2] * deltat;
  
  // Update euler angles
  euler_rad[0] = (euler.angle.roll * DEG_TO_RAD);
  euler_rad[1] = (euler.angle.pitch * DEG_TO_RAD);
  euler_rad[2] = (euler.angle.yaw * DEG_TO_RAD);

  // Update quaternion
  quaternians[0] = quaternion.element.w;
  quaternians[1] = quaternion.element.x;
  quaternians[2] = quaternion.element.y;
  quaternians[3] = quaternion.element.z;

  // Update angular velocity
  velocity[0] = angvel.axis.x;
  velocity[1] = angvel.axis.y;
  velocity[2] = angvel.axis.z;

}

// initialise the IMU
void IMU::init() {
  pinMode(AD0_PIN, OUTPUT);
  digitalWrite(AD0_PIN, HIGH);

  MPU.begin();

  // setting the accelerometer full scale range to +/-8G 
  MPU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  MPU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  MPU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  // setting SRD to 19 for a 50 Hz update rate
  MPU.setSrd(9);
  time_former = micros();

  #ifdef VL53L0X_CONNECT

    // Disable/reset all sensors by driving their XSHUT pins low.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
      pinMode(xshutPins[i], OUTPUT);
      digitalWrite(xshutPins[i], LOW);
    }

    // Enable, initialize, and start each sensor, one by one.
    for (uint8_t i = 0; i < sensorCount; i++)
    {
      // Stop driving this sensor's XSHUT low. This should allow the carrier
      // board to pull it high. (We do NOT want to drive XSHUT high since it is
      // not level shifted.) Then wait a bit for the sensor to start up.
      pinMode(xshutPins[i], INPUT);
      delay(10);

      sensors[i].setTimeout(500);
      if (!sensors[i].init())
      {
        Serial.print("Failed to detect and initialize sensor ");
        Serial.println(i);
        while (1);
      }

      // Each sensor must have its address changed to a unique value other than
      // the default of 0x29 (except for the last one, which could be left at
      // the default). To make it simple, we'll just count up from 0x2A.
      sensors[i].setAddress(0x2A + i);

      sensors[i].startContinuous(50);
    }

  #endif //VL53L0X_ADDRESS

  FusionOffsetInitialise(&offset, SAMPLE_RATE);
  FusionAhrsInitialise(&ahrs);

  // Set AHRS algorithm settings
  const FusionAhrsSettings settings = {
          .convention = FusionConventionNed,
          .gain = 0.7f,
          .gyroscopeRange = 500.0f, /* replace this with actual gyroscope range in degrees/s */
          .accelerationRejection = 10.0f,
          .magneticRejection = 10.0f,
          .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
  };
  FusionAhrsSetSettings(&ahrs, &settings);

}

// Get euler angles
void IMU::getEulerRad(double* roll, double* pitch, double* yaw) {
  *roll = euler_rad[0];
  *pitch = euler_rad[1];
  *yaw = euler_rad[2];
}

// Get quaternians
void IMU::getQuaternians(double* w, double* x, double* y, double* z) {
  *w = quaternians[0];
  *x = quaternians[1];
  *y = quaternians[2];
  *z = quaternians[3];
}

// Get position
void IMU::getPos(double* x, double* y, double* z) {
  *x = position[0];
  *y = position[1];
  *z = position[2];
}

// get lidar data
void IMU::getLidarData(double* data1, double* data2) {
  #ifdef VL53L0X_CONNECT

    *data1 = sensors[0].readRangeContinuousMillimeters();
    *data2 = 0;//sensors[1].readRangeContinuousMillimeters();

  #endif //VL53L0X_ADDRESS
}

// Get angular velocity
void IMU::getAngularVelocity(double* x, double* y, double* z) {
  *x = angular_velocity[0];
  *y = angular_velocity[1];
  *z = angular_velocity[2];
}

// Get yaw
void IMU::getYaw(double* yaw) {
  *yaw = euler_rad[2];
}