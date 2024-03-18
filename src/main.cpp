#include <WiFi.h>
#include <ESP32Servo.h>
#include "freertos/task.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <Wire.h>
#include <Arduino.h>

const char* ssid = "TDC-C0A4";
const char* password = "3356f79c4";
IPAddress local_ip(192,168,1,52);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
WiFiServer server(80);

//Gyro and Accel
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
  double z = 0;
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  std::tuple<float, float, float, float> getCurrentValues() const {
        return std::make_tuple(z, roll, pitch, yaw);
  }
};
Angle angle;

struct Mag{
  double x = 0;
  double y = 0;
  double z = 0;
  double calX = 0; double calY = 0; double calZ = 0;
};  
Mag mag;

// class for the MPU6050 and HMC5883L
MPU6050 accelgyro;
HMC5883L magnometer;

/*------------------------------- FUNCTIONS ----------------------------*/
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

// Function to get the angle
Angle getAngle() {

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&raw.ax, &raw.ay, &raw.az, &raw.gx, &raw.gy, &raw.gz);

  // read raw magnetometer measurements from device
  magnometer.getHeading(&raw.mx, &raw.my, &raw.mz);

  // apply the complementary filter
  angle = complementaryFilter(angle, raw, mag);

  return angle;
}

//
int x = 0;
int y = 0;
int z = 50;
int buzzer = 12;
int counter = 0;
Servo esc1;
Servo esc2;
Servo esc3;

const int maxClients = 5; // Maximum number of clients the server can handle
WiFiClient clients[maxClients]; // Array to hold the client instances


void updateESC1(int z) {
  esc1.write(z);
}

void updateESC2(int z) {
  esc2.write(z);
}

void updateESC3(int z) {
  esc3.write(z);
}
bool run = true;
bool kill_switch = true;
void processCommand(const char command) {
  switch (command) {
    case 'w':
      y += 1;
      //updateESC1(y);
      break;
    case 's':
      y -= 1;
      //updateESC1(y);
      break;
    case 'a':
      x -= 1;
      //updateESC2(x);
      break;
    case 'd':
      x += 1;
      //updateESC2(x);
      break;
    case 'r':
      z += 4;
      //updateESC3(z);
      break;
    case 'f':
      z -= 4;
      //updateESC3(z);
      break;
    case 'm':
      run = true;
      kill_switch = true;
      break;
    case 'l':
      updateESC1(0);
      updateESC2(0);
      updateESC3(0);
      run = false;
      kill_switch = false;
      break;
    default:
      break;
  }
}


Angle getGyroInfo2(){
  Angle angle;
  angle.roll=0.0;
  angle.pitch=20.0;
  angle.yaw=0.0;
  return angle;
}
float in_voltage;

void checkBatteryVoltage(){
  in_voltage = (analogRead(34)/3722.0)*12.6;
  //in_voltage = map(in_voltage, 0.0, 3722.0,0.0, 12.6);
  //Serial.println(voltage);
  
  if (in_voltage < 8.7){
    tone(buzzer, 1000);
    Serial.println(in_voltage);
    run = false;
    kill_switch = false;
  }
  else
  {
    noTone(buzzer);
    Serial.println(in_voltage);
  }
}
// control variables
float roll_buffer[10];
float pitch_buffer[10];
float yaw_buffer[10];
float target_x = 0.0;
float target_y = 0.0;
float target_z = 0.0;
float target_altitude = 5.0;
float current_x = 0.0;
float current_y = 0.0;
float current_z = 0.0;
float current_altitude = 0.0;
float error_x = 0.0;
float error_y = 0.0;
float error_z = 0.0;
float error_altitude = 0.0;
float kp = 1.5;
float ki = 0.10;  // Integral gain //08
float kd = 0.13; // Derivative gain //0.10
float U_alti = 0.0;
float U_r1 = 0.0;
float U_r2 = 0.0;
float U_p1 = 0.0;
float U_p2 = 0.0;
float U_p3 = 0.0;
float output1 = 0.0;
float output2 = 0.0;
float output3 = 0.0;
float real_output1 = 0.0;
float real_output2 = 0.0;
float real_output3 = 0.0;
float kp_2 = 0.20; // 25
float integral_x = 0.0;  // Integral term for x
float integral_y = 0.0;  // Integral term for y
float derivative_x = 0.0; // Derivative term for x
float derivative_y = 0.0; // Derivative term for y
float previous_error_x = 0.0; // Previous error for x
float previous_error_y = 0.0; // Previous error for y
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

void control(){
  unsigned long currentTime = millis();
  elapsedTime = currentTime - previousTime;
  previousTime = currentTime;

  Angle current = getAngle();
  
  // Update roll, pitch, and yaw buffers
  roll_buffer[9] = current.roll;
  pitch_buffer[9] = current.pitch;
  yaw_buffer[9] = current.yaw;

  // Shift the buffer
  for (int i = 0; i < 9; i++){
    roll_buffer[i] = roll_buffer[i+1];
    pitch_buffer[i] = pitch_buffer[i+1];
    yaw_buffer[i] = yaw_buffer[i+1];
  }

  // Calculate current rotations as the mean of the buffer
  current_x = (roll_buffer[0] + roll_buffer[1] + roll_buffer[2] +pitch_buffer[3]+ pitch_buffer[4]+pitch_buffer[5]+roll_buffer[6]+roll_buffer[7]+roll_buffer[8]+roll_buffer[9])/10;
  current_y = (pitch_buffer[0] + pitch_buffer[1] + pitch_buffer[2] + roll_buffer[3]+ roll_buffer[4]+roll_buffer[5]+pitch_buffer[6]+pitch_buffer[7]+pitch_buffer[8]+pitch_buffer[9])/10;
  current_z = (yaw_buffer[0] + yaw_buffer[1] + yaw_buffer[2] + yaw_buffer[3]+ yaw_buffer[4]+yaw_buffer[5]+yaw_buffer[6]+yaw_buffer[7]+yaw_buffer[8]+yaw_buffer[9])/10;

  // Set target positions
  target_altitude = z;
  target_x = x;
  target_y = y;

  // Calculate errors
  error_x = target_x - current_x;
  error_y = target_y - current_y;
  error_z = target_z - current_z;
  error_altitude = target_altitude - current_altitude;

  // Update integral terms
  integral_x += error_x * elapsedTime;
  integral_y += error_y * elapsedTime;
  // constrain integral terms
  integral_x = constrain(integral_x, -50, 50);
  integral_y = constrain(integral_y, -50, 50);


  // Calculate control inputs
  U_alti = kp * error_altitude;
  U_r1 = kp_2 * error_x + ki * integral_x;
  U_r2 = -kp_2 * error_x + ki * integral_x;
  U_p1 = kp_2 * error_y + ki * integral_y;
  U_p2 = kp_2 * error_y + ki * integral_y;
  U_p3 = -kp_2 * error_y + ki * integral_y;

  // Calculate real outputs
  if (run && kill_switch){
    output1 = 6 + U_alti + U_r1 + U_p1;
    output2 = 6 + U_alti + U_r2 + U_p2;
    output3 = 5 + U_alti + U_p3;
  }
  else{
    output1 = 0;
    output2 = 0;
    output3 = 0;
  }

  // Constrain outputs
  output1 = constrain(output1, 0, 140);
  output2 = constrain(output2, 0, 140);
  output3 = constrain(output3, 0, 140); 

  // Update real outputs and ESCs
  real_output1 = output1;
  real_output2 = output2;
  real_output3 = output3;
  updateESC1(output1);
  updateESC2(output2);
  updateESC3(output3);
}
void com(){
  bool anyClientConnected = false;  // Flag to track if any client is connected

  for (int i = 0; i < maxClients; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i] = server.available();
      if (clients[i]) {
        Serial.println("New client connected");
        //clients[i].println("Welcome to the server!");
      }
    } else {
      anyClientConnected = true;  // Set the flag if any client is connected
    }
  }

  // Update run variable based on client connection status
  run = anyClientConnected;

  // Handle client messages
  for (int i = 0; i < maxClients; i++) {
    if (clients[i] && clients[i].connected() && clients[i].available()) {
      String message = clients[i].readStringUntil('\n');
      Serial.print("Client ");
      Serial.print(i);
      Serial.print(" sent: ");
      Serial.println(message);
      char c = message[0];
      if (sizeof(c)  == 1){
          processCommand(c); // Process the received command
      }
    }
  }
  int pos[3]={x,y,z};
  clients[0].print(error_x); clients[0].print(":"); clients[0].print(error_y);clients[0].print(":"); clients[0].print(output1); clients[0].print(":"); clients[0].print(output2); clients[0].print(":"); clients[0].print(output3);clients[0].print("n"); 
  clients[1].print(error_x); clients[1].print(":"); clients[1].print(error_y);clients[1].print(":"); clients[1].print(output1); clients[1].print(":"); clients[1].print(output2); clients[1].print(":"); clients[1].print(output3);clients[1].print("n");
  clients[2].print(error_x); clients[2].print(":"); clients[2].print(error_y);clients[2].print(":"); clients[2].print(output1); clients[2].print(":"); clients[2].print(output2); clients[2].print(":"); clients[2].print(output3);clients[2].print("n");
  clients[3].print(error_x); clients[3].print(":"); clients[3].print(error_y);clients[3].print(":"); clients[3].print(output1); clients[3].print(":"); clients[3].print(output2); clients[3].print(":"); clients[3].print(output3);clients[3].print("n");
  clients[4].print(error_x); clients[4].print(":"); clients[4].print(error_y);clients[4].print(":"); clients[4].print(output1); clients[4].print(":"); clients[4].print(output2); clients[4].print(":"); clients[4].print(output3);clients[4].print("n");
}
void comTask(void *pvParameters) {
  while (1) {
    // Your communication task code here
    // This will run indefinitely
    com();
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 100 milliseconds
  }
}
void checkBatteryVoltageTask(void *pvParameters) {
  while (1) {
    // Your communication task code here
    // This will run indefinitely
    checkBatteryVoltage();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 100 milliseconds
  }
}
void controlTask(void *pvParameters) {
  while (1) {
    // Your control task code here
    // This will run indefinitely
    control();
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 100 milliseconds
  }
}


void setup() {
  Wire.begin();
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
  magnometer.initialize();

  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output
  tone(buzzer, 4000);

  Serial.begin(115200);
  delay(1000);

  WiFi.config(local_ip, gateway, subnet);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting...");
  }
  Serial.println("Connected to WiFi");

  server.begin();
  Serial.println("Server started");

  esc1.attach(32, 1000, 2000);
  esc2.attach(33, 1000, 2000);
  esc3.attach(25, 1000, 2000);

  esc1.write(0);
  esc2.write(0);
  esc3.write(0);

  // add delay to allow the ESCs to initialize
  delay(9000);
  noTone(buzzer);

  xTaskCreatePinnedToCore(
    comTask,          // Task function
    "Communication", // Name of task
    10000,            // Stack size of task
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL,             // Task handle.
    0                 // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    controlTask,     // Task function
    "Control",       // Name of task
    10000,           // Stack size of task
    NULL,            // Task input parameter
    1,               // Priority of the task
    NULL,            // Task handle.
    1                // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    checkBatteryVoltageTask,     // Task function
    "CheckBatteryVoltage",       // Name of task
    10000,           // Stack size of task
    NULL,            // Task input parameter
    1,               // Priority of the task
    NULL,            // Task handle.
    0                // Core where the task should run
  ); 
}

void loop() {
  // Empty, since we are using FreeRTOS tasks
}
