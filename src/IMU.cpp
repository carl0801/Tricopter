#include "IMU.h"

// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.03f, 0.0f, -0.005f};
const FusionMatrix softIronMatrix = {0.929729, -0.00859594, -0.0070703, -0.00859594, 0.987421, -0.00911484, -0.0070703, -0.00911484, 0.993384};
const FusionVector hardIronOffset = {5.76039, -12.441, 12.208};


// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

MPU9250 MPU(Wire, MPU9250_ADDRESS);


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

// Update the IMU
void IMU::update_IMU() {

  //Read sensor data from MPU9250
  read_sensors();

  // Calculate time since last loop
  time_now = micros();
  deltat = (float)(time_now - time_former) / 1000000.0f;
  time_former = time_now;
  
  FusionVector gyroscope = {-gyro[1], -gyro[0], -gyro[2]}; // replace this with actual gyroscope data in degrees/s
  FusionVector accelerometer = {accel[1], accel[0], accel[2]}; // replace this with actual accelerometer data in g
  FusionVector magnetometer = {magnetom[1], magnetom[0], -magnetom[2]}; // replace this with actual magnetometer data in arbitrary units

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
  const FusionEuler euler = FusionQuaternionToEuler(quaternion);
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  // Update position
  position[0] = earth.axis.x * GRAVITY;
  position[1] = earth.axis.y * GRAVITY;
  position[2] = earth.axis.z * GRAVITY;

  // Update euler angles
  euler_rad[0] = (euler.angle.roll * DEG_TO_RAD) ;//- 3.1f;
  euler_rad[1] = (euler.angle.pitch * DEG_TO_RAD) ;//- 0.03f;
  euler_rad[2] = (euler.angle.yaw * DEG_TO_RAD) - yaw_offset;

  // Update quaternion
  quaternians[0] = quaternion.element.w;
  quaternians[1] = quaternion.element.x;
  quaternians[2] = quaternion.element.y;
  quaternians[3] = quaternion.element.z;

  // Delay to maintain sample rate
  delayMicroseconds(1/SAMPLE_RATE * 1000000);
}

// initialise the IMU
void IMU::init_IMU() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

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

  // calculate yaw offset
  /*float offset = 0;
  for (int i = 0; i < (SAMPLE_RATE * 2); i++) {
    update_IMU();
    offset += euler_rad[2];
  }
  yaw_offset = offset / (SAMPLE_RATE * 2);*/

}

// Get euler angles
void IMU::getEulerRad(float* roll, float* pitch, float* yaw) {
  *roll = euler_rad[0];
  *pitch = euler_rad[1];
  *yaw = euler_rad[2];
}

// Get quaternians
void IMU::getQuaternians(float* w, float* x, float* y, float* z) {
  *w = quaternians[0];
  *x = quaternians[1];
  *y = quaternians[2];
  *z = quaternians[3];
}

// Get position
void IMU::getPosition(float* x, float* y, float* z) {
  *x = position[0];
  *y = position[1];
  *z = position[2];
}
