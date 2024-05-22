#include "IMU.h" 
#include <Arduino.h> 

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
//const FusionVector gyroscopeOffset = {-0.0008878f, 0.0002957f, 0.0000198f};
const FusionVector gyroscopeOffset = {-3.39f,2.70f,-0.10f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
//const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionVector accelerometerOffset = {0.01f,0.02f,0.04f};
//const FusionMatrix softIronMatrix = {0.98678, -0.00660813, 0.0348315, -0.00660813, 0.967686, 0.0273002, 0.0348315, 0.0273002, 0.904855};
const FusionMatrix softIronMatrix = {0.93,0,0,0,1.02,0,0,0,1.06};
//const FusionVector hardIronOffset = {864.903, -275.123, -64.4907};
const FusionVector hardIronOffset = {218.0,-270.5,-57.9};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

MPU9250 MPU(MPU9250_ADDRESS, I2Cport, I2Cclock);


#ifdef VL53L0X_CONNECT
  // The number of sensors in your system.
  const uint8_t sensorCount = 1;

  // The Arduino pin connected to the XSHUT pin of each sensor.
  const uint8_t xshutPins[sensorCount] = { 4 }; 

  VL53L0X sensors[sensorCount];

#endif //VL53L0X_ADDRESS

IMU::IMU(double dt): 
  SAMPLE_RATE(static_cast<u_int>(1.0 / (dt/1000)))
{}

void IMU::sendToPC(float* data1, float* data2, float* data3)
{
  byte* byteData1 = (byte*)(data1);
  byte* byteData2 = (byte*)(data2);
  byte* byteData3 = (byte*)(data3);
  byte buf[12] = {byteData1[0], byteData1[1], byteData1[2], byteData1[3],
                 byteData2[0], byteData2[1], byteData2[2], byteData2[3],
                 byteData3[0], byteData3[1], byteData3[2], byteData3[3]};
  Serial.write(buf, 12);
}

// Read data from MPU9250
void IMU::read_sensors() {

  // If data ready bit set, all data registers have new data
  if (MPU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    MPU.readAccelData(MPU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    MPU.ax = (float)MPU.accelCount[0] * MPU.aRes; 
    MPU.ay = (float)MPU.accelCount[1] * MPU.aRes; 
    MPU.az = (float)MPU.accelCount[2] * MPU.aRes; 

    MPU.readGyroData(MPU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    MPU.gx = (float)MPU.gyroCount[0] * MPU.gRes;
    MPU.gy = (float)MPU.gyroCount[1] * MPU.gRes;
    MPU.gz = (float)MPU.gyroCount[2] * MPU.gRes;

    MPU.readMagData(MPU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    MPU.mx = (float)MPU.magCount[0] * MPU.mRes;
    MPU.my = (float)MPU.magCount[1] * MPU.mRes;
    MPU.mz = (float)MPU.magCount[2] * MPU.mRes;

    //sendToPC(&MPU.mx, &MPU.my, &MPU.mz);

    //printf("Mag: %0.6f %0.6f %0.6f\n", MPU.mx, MPU.my, MPU.mz);

  //printf("Accel: %0.6f %0.6f %0.6f Gyro: %0.6f %0.6f %0.6f Mag: %0.6f %0.6f %0.6f\n", -MPU.ax, -MPU.ay, -MPU.az, MPU.gx, MPU.gy, MPU.gz, MPU.mx, MPU.my, MPU.mz);
  } 

}

// Update the IMU
void IMU::update_IMU() {

  //Read sensor data from MPU9250
  read_sensors();
  //IMU::sendToPC(&magnetom[0], &magnetom[1], &magnetom[2]);
  // Calculate time since last loop
  time_now = micros();
  deltat = (double)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  
  FusionVector gyroscope = {MPU.gx, MPU.gy, MPU.gz};    
  FusionVector accelerometer = {MPU.ax, MPU.ay, MPU.az}; 
  FusionVector magnetometer = {MPU.mx, MPU.my, MPU.mz};

  // Apply calibration
  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
  magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

  

  // Update gyroscope offset correction algorithm
  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // change axis to align to dynamics
  gyroscope = {gyroscope.axis.x, -gyroscope.axis.y, -gyroscope.axis.z};
  //accelerometer = {accelerometer.axis.x, -accelerometer.axis.y, -accelerometer.axis.z};
  accelerometer = {-accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z};
  magnetometer = {magnetometer.axis.y, -magnetometer.axis.x, magnetometer.axis.z};

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
  euler_rad[0] = (euler.angle.roll);
  euler_rad[1] = (euler.angle.pitch);
  euler_rad[2] = (euler.angle.yaw);

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
  
  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(INT_PIN, INPUT);
  digitalWrite(INT_PIN, LOW);
  pinMode(AD0_PIN, OUTPUT);
  digitalWrite(AD0_PIN, HIGH);

  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed

  MPU.MPU9250SelfTest(MPU.selfTest);
  //MPU.calibrateMPU9250(MPU.gyroBias, MPU.accelBias);

  MPU.initMPU9250();
  MPU.initAK8963(MPU.factoryMagCalibration);

  // Get sensor resolutions, only need to do this once
  MPU.getAres();
  MPU.getGres();
  MPU.getMres();
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
        //Serial.print("Failed to detect and initialize sensor ");
        //Serial.println(i);
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
          .convention = FusionConventionNwu,
          .gain = 0.5f, //0.5f
          .gyroscopeRange = MPU.gRes * 32768.0f, /* replace this with actual gyroscope range in degrees/s */
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

    // Read the measured height from the sensor
    uint16_t measuredHeight = sensors[0].readRangeContinuousMillimeters();

    // Convert the measured height to a 3D point in the sensor's coordinate system
    Eigen::Vector3d pointInSensorCoordinates(0, 0, measuredHeight);

    // Get the current orientation quaternion
    Eigen::Quaterniond q;
    q.w() = quaternians[0];
    q.x() = quaternians[1];
    q.y() = quaternians[2];
    q.z() = quaternians[3];

    // Calculate the inverse of the current orientation quaternion
    Eigen::Quaterniond q_inv = q.inverse();

    // Rotate the point by the inverse quaternion to get the point in the world's coordinate system
    Eigen::Vector3d pointInWorldCoordinates = q_inv * pointInSensorCoordinates;

    // The z-coordinate of the point in the world's coordinate system is the tilt-compensated height
    double tiltCompensatedHeight = pointInWorldCoordinates.z();

    // Return the compensated height
    *data1 = tiltCompensatedHeight;
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

void IMU::getMagnetom(float* m1, float* m2, float* m3) {
  *m1 = magnetom[0];
  *m2 = magnetom[1];
  *m3 = magnetom[2];
}

void IMU::getAcc(float* m1, float* m2, float* m3) {
  *m1 = accel[0];
  *m2 = accel[1];
  *m3 = accel[2];
}

void IMU::getGyro(float* m1, float* m2, float* m3) {
  *m1 = gyro[0];
  *m2 = gyro[1];
  *m3 = gyro[2];
}


