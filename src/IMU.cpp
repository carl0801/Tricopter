
#include "IMU.h"

/*------------------------------- CLASSES -----------------------------*/

#ifdef HW_290_ADDRESS
  
    // class for the MPU6050
    MPU6050 accelgyro;

    // class for the HMC5883L
    HMC5883L magnometer;

    // class for BMP085
    Adafruit_BMP085 bmp;

#endif //HW_290_ADDRESS

#ifdef MPU9250_ADDRESS
  
    // class for the MPU9250
    MPU9250 mpu;

#endif //MPU9250_ADDRESS

#ifdef VL53L0X_ADDRESS

    // class for vl53l0x
    VL53L0X lidar;

#endif //VL53L0X_ADDRESS

// class for the AHRS algorithm
FusionAhrs ahrs;


/*------------------------------- FUNCTIONS ----------------------------*/

// Initialize the sensors
void IMU::init_sensors() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  #ifdef HW_290_ADDRESS

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

  #endif //HW_290_ADDRESS

  #ifdef MPU9250_ADDRESS

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);

    delay(2000);

    if (!mpu.setup(0x69)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();
    mpu.verbose(false);

  #endif //MPU9250_ADDRESS


  // initialize the ToF sensor
  #ifdef VL53L0X_ADDRESS

    lidar.setTimeout(500);
    if (!lidar.init())
    {
      Serial.println("Failed to detect and initialize sensor!");
      while (1) {}
    }
    lidar.startContinuous();

  #endif //VL53L0X_ADDRESS

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
void IMU::transformation()   
{
  float calibrated_values[3];
  float uncalibrated_values[3];

  #ifdef HW_290_ADDRESS

    uncalibrated_values[3] = data.mx, data.my, data.mz;

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

    data.mx = calibrated_values[0];
    data.my = calibrated_values[1];
    data.mz = calibrated_values[2];

  #endif //HW_290_ADDRESS

  #ifdef MPU9250_ADDRESS

    uncalibrated_values[3] = data.mx1, data.my1, data.mz1;

    //replace M11, M12,..,M33 with your transformation matrix data
    double calibration_matrix[3][3] = {
        {1.755, 0.076, 0.154},
        {-0.151, 1.567, -0.067},
        {-0.196, 0.125, 1.314}
    };
    //bias[3] is the bias
    //replace Bx, By, Bz with your bias data
    double bias[3] = 
    {
      -113.433,
      -457.751,
      162.402
    };   

    //calculation
    for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
    float result[3] = {0, 0, 0};
    for (int i=0; i<3; ++i)
      for (int j=0; j<3; ++j)
        result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
    for (int i=0; i<3; ++i) calibrated_values[i] = result[i];

    data.mx1 = calibrated_values[0];
    data.my1 = calibrated_values[1];
    data.mz1 = calibrated_values[2];

  #endif //HW_290_ADDRESS

}

// Read the sensors
void IMU::readSensor() {
    
    #ifdef HW_290_ADDRESS

      // read raw accel/gyro measurements from device
      accelgyro.getMotion6(&raw.ax, &raw.ay, &raw.az, &raw.gx, &raw.gy, &raw.gz);

      // read raw magnetometer measurements from device
      magnometer.getHeading(&raw.mx, &raw.my, &raw.mz);
    
    #endif //HW_290_ADDRESS
    
    #ifdef MPU9250_ADDRESS

      mpu.update();

    #endif //MPU9250_ADDRESS


}

// Get the current reading from the sensors
void IMU::getRead(){

  #ifdef HW_290_ADDRESS

    data.gx = raw.gx / 131;
    data.gy = raw.gy / 131;
    data.gz = raw.gz / 131;

    data.ax = raw.ax / 16384.0;
    data.ay = raw.ay / 16384.0;
    data.az = raw.az / 16384.0;

    data.mx = raw.mx;
    data.my = raw.my;
    data.mz = raw.mz;
    
  #endif //HW_290_ADDRESS

  #ifdef MPU9250_ADDRESS
  
    data.gx1 = mpu.getGyroX();
    data.gy1 = mpu.getGyroY();
    data.gz1 = mpu.getGyroZ();
    data.ax1 = mpu.getAccX();
    data.ay1 = mpu.getAccY();
    data.az1 = mpu.getAccZ();
    data.mx1 = mpu.getMagX();
    data.my1 = mpu.getMagY();
    data.mz1 = mpu.getMagZ();

  #endif //MPU9250_ADDRESS
  
  // Apply the calibration matrix to the magnetometer
  transformation();

}

//Get euler rotation
void IMU::getEulerRotation(double *roll, double *pitch, double *yaw) {

  getRead();

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
  getRead();

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
  #if defined(VL53L0X_ADDRESS)
    *altitude = lidar.readRangeContinuousMillimeters();
  #else
    *altitude = 0;
  #endif //VL53L0X_ADDRESS

};

// Get the pressure from the BMP085 (Pa)
void IMU::getPressure(double *pressure) {
  #if defined(HW_290_ADDRESS) 
    *pressure = bmp.readPressure();
  #else
    *pressure = 0;

  #endif //HW_290_ADDRESS

}

// Get the temperature from the BMP085 (C°)
void IMU::getTemperature(double *temperature) {
  #if defined(HW_290_ADDRESS) 
    *temperature = bmp.readTemperature();
  #else
    *temperature = 0;
  #endif //HW_290_ADDRESS
}

// Test output from sensors by returning all sensors data
void IMU::test(float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float *mx, float *my, float *mz, int board){
  
  getRead();

  #if defined(HW_290_ADDRESS) 
    if(board == HW_290_ADDRESS){
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
  #endif //HW_290_ADDRESS

  #if defined(MPU9250_ADDRESS) 
    if(board == MPU9250_ADDRESS){
      *gx = data.gx1;
      *gy = data.gy1;
      *gz = data.gz1;
      *ax = data.ax1;
      *ay = data.ay1;
      *az = data.az1;
      *mx = data.mx1;
      *my = data.my1;
      *mz = data.mz1;
    }
  #endif //MPU9250_ADDRESS

}
