#include <Arduino.h>
#include <FlightController.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <IPAddress.h>
#include "freertos/task.h"
#include "BLE_COMMS.cpp"




// Position, Orientation, Linear Acceleration, Angular Acceleration, Goal Position, Goal Orientation
#define NAVIGATION_SERVICE_UUID       "eee14234-a127-4cc3-9c8f-522fbb2d8df1"
// Servo Angle, Motor PWM
#define CONTROL_SERVICE_UUID  "bfff9825-156a-4a57-a0a3-700ed8378465"
// Lidar Z, Lidar XY, Time, Battery
#define SENSOR_SERVICE_UUID      "081e633b-76b3-497c-8161-af640c8debff"

#define POSITION_CHARACTERISTIC_UUID   "f35ce007-92ef-4241-b965-8b68c23a0ac6"
#define ORIENTATION_CHARACTERISTIC_UUID   "ff75d773-3e8e-4085-96e4-6a72a54a1160"

#define LINEAR_ACCELERATION_CHARACTERISTIC_UUID   "d81d39b2-cd9f-4cb2-8a36-8a6e8e04fa0d"
#define ANGULAR_ACCELERATION_CHARACTERISTIC_UUID   "56145bbf-ec14-44a6-83c0-b773b01b1a21"

#define GOAL_POSITION_CHARACTERISTIC_UUID   "5b31a140-65aa-480a-914b-06461f262c6c"
#define GOAL_ORIENTATION_CHARACTERISTIC_UUID   "9bf6ae34-bbb3-43e4-b25a-80ec6bf9e5a2"

#define SERVO_ANGLE_CHARACTERISTIC_UUID   "09abb19e-dbc1-4d00-b137-7f3e5ebf6b61"
#define MOTOR_PWM_CHARACTERISTIC_UUID   "c9d16ab4-db14-4e77-bcbb-2825fe7d08c9"

#define LIDAR_Z_CHARACTERISTIC_UUID   "59f8114c-5335-4643-82b8-6774611fddb1"
#define LIDAR_XY_CHARACTERISTIC_UUID   "beb126e2-0cf9-433b-91c2-6a9c5597fb2d"

#define TIME_CHARACTERISTIC_UUID   "3d893a39-a3fa-45bd-a23b-337cec8156e6"
#define BATTERY_CHARACTERISTIC_UUID   "fabb9266-b2e1-4b16-90c0-7c9a6533189e"
#define POWER_USAGE_CHARACTERISTIC_UUID   "3355dd98-75f2-49a6-83f2-812d3fbcafbd"





bool run = true;
const char* ssid = "TDC-C0A4";
const char* password = "3356f79c4";
IPAddress local_ip(192,168,1,51);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
WiFiServer server(80);


const double STEP_TIME = 5; // timeInterval for flight controller
//const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
//const double ALPHA = 0.8; // alpha value for the complementary filter


Servo esc1;
Servo esc2;
Servo esc3;

motorData motorValues;


double x = 0;
double y = 0;
double emils_z = 0;




const int maxClients = 5; // Maximum number of clients the server can handle
WiFiClient clients[maxClients]; // Array to hold the client instances

void setupBLE() {
  BLEDevice::init("Tricopter_FlightController");

  pServer = BLEDevice::createServer();

  // Create Services
  BLEService* navigationService = createService(NAVIGATION_SERVICE_UUID);
  BLEService* controlService = createService(CONTROL_SERVICE_UUID);
  BLEService* sensorService = createService(SENSOR_SERVICE_UUID);

  // Navigation Service Characteristics
  BLECharacteristic* positionCharacteristic = createCharacteristicRN(navigationService, POSITION_CHARACTERISTIC_UUID);
  BLECharacteristic* orientationCharacteristic = createCharacteristicRN(navigationService, ORIENTATION_CHARACTERISTIC_UUID);
  BLECharacteristic* linearAccelerationCharacteristic = createCharacteristicRN(navigationService, LINEAR_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic* angularAccelerationCharacteristic = createCharacteristicRN(navigationService, ANGULAR_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic* goalPositionCharacteristic = createCharacteristicRWN(navigationService, GOAL_POSITION_CHARACTERISTIC_UUID);
  BLECharacteristic* goalOrientationCharacteristic = createCharacteristicRWN(navigationService, GOAL_ORIENTATION_CHARACTERISTIC_UUID);

  // Control Service Characteristics
  BLECharacteristic* servoAngleCharacteristic = createCharacteristicRWN(controlService, SERVO_ANGLE_CHARACTERISTIC_UUID);
  BLECharacteristic* motorPWMCharacteristic = createCharacteristicRWN(controlService, MOTOR_PWM_CHARACTERISTIC_UUID);

  // Sensor Service Characteristics
  BLECharacteristic* lidarZCharacteristic = createCharacteristicRN(sensorService, LIDAR_Z_CHARACTERISTIC_UUID);
  BLECharacteristic* lidarXYCharacteristic = createCharacteristicRN(sensorService, LIDAR_XY_CHARACTERISTIC_UUID);
  BLECharacteristic* timeCharacteristic = createCharacteristicRN(sensorService, TIME_CHARACTERISTIC_UUID);
  BLECharacteristic* batteryCharacteristic = createCharacteristicRN(sensorService, BATTERY_CHARACTERISTIC_UUID);
  BLECharacteristic* powerUsageCharacteristic = createCharacteristicRN(sensorService, POWER_USAGE_CHARACTERISTIC_UUID);

  // Set callback for characteristic write (implement the logic in CharacteristicCallbacks class)
  CharacteristicCallbacks characteristicCallback;
  goalPositionCharacteristic->setCallbacks(&characteristicCallback);
  goalOrientationCharacteristic->setCallbacks(&characteristicCallback);
  servoAngleCharacteristic->setCallbacks(&characteristicCallback);
  motorPWMCharacteristic->setCallbacks(&characteristicCallback);

  // Start advertising the server
  pServer->setCallbacks(new ServerCallbacks());
  pServer->startAdvertising();
  Serial.println("BLE Server started advertising");
}


FlightController flightController(STEP_TIME);
IMU& imu = flightController.imu;
void updateMotor(motorData motorValues) {

  //make sure the value is max 180
  motorValues.omega_1 = std::min((motorValues.omega_1 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_2 = std::min((motorValues.omega_2 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_3 = std::min((motorValues.omega_3 - 182.69 ) / 843.09 * 180, 90.0);

  //constrain the value to be between 0 and 180
  motorValues.alpha_1 = constrain(motorValues.alpha_1*180/M_PI + 90, 0, 180);
  motorValues.alpha_2 = constrain(motorValues.alpha_2*180/M_PI + 90, 0, 180);
  motorValues.alpha_3 = constrain(motorValues.alpha_3*180/M_PI + 90, 0, 180);

  /* Serial.print("omega_1: ");
  Serial.print(motorValues.omega_1);
  Serial.print(" omega_2: ");
  Serial.print(motorValues.omega_2);
  Serial.print(" omega_3: ");
  Serial.println(motorValues.omega_3); */


  esc1.write(motorValues.omega_1);
  esc2.write(motorValues.omega_2);
  esc3.write(motorValues.omega_3);
  /*
  servo1.write(motorValues.alpha_1*180/M_PI)
  servo2.write(motorValues.alpha_2*180/M_PI)
  servo3.write(motorValues.alpha_3*180/M_PI)
  
  */


}



void control(double yawOffset){
  //flightController.calculate();
  //motorData data = flightController.getMotorData();
  updateMotor(flightController.calculate(double(yawOffset)));
}

void com(void *pvParameters) {
  struct data* sensorData = (struct data*) pvParameters;

  while (1) {
    // Update Navigation Service Characteristics
    notifyFloatBLE(positionCharacteristic, &(sensorData->x));  // Send x, y, z
    notifyFloatBLE(orientationCharacteristic, &(sensorData->yaw));  // Send yaw, roll, pitch
    notifyDoubleBLE(linearAccelerationCharacteristic, &(sensorData->w));  // Send w

    // Update Sensor Service Characteristics (assuming omega values are doubles)
    notifyDoubleBLE(lidarZCharacteristic, &(sensorData->lidar2));
    notifyDoubleBLE(angularAccelerationCharacteristic, &(sensorData->omega_1));  // Send omega_1, omega_2, omega_3
    notifyDoubleBLE(timeCharacteristic, &(sensorData->alpha_1));  // Assuming time is stored in alpha_1

    // Other service updates can be added here based on your needs

    // Implement a delay or use a timer to control update rate
    vTaskDelay(pdMS_TO_ticks(10)); // Update every 10 milliseconds (adjust as needed)
  }
}

double voltage;
//float realVoltage;

double getBatteryVoltage(){
  //max input is 3.0V and max output should be 12.6V
  voltage = (analogRead(34)/ 3722.727273) * 12.6;

  //Serial.println(voltage);
  return voltage;
}

void comTask(void *pvParameters) {
  /* while (run) {
    // Your communication task code here
    // This will run indefinitely
    com();
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 100 milliseconds
  }
  vTaskDelete(NULL); */
  while (1) {
    // Your communication task code here
    // This will run indefinitely
    com();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
  }
  vTaskDelete(NULL);
}




void controlTask(void *pvParameters) {
  unsigned long startTime;
  unsigned long endTime;
  double yawOffset = 0;
  double placeholder = 0;
  double placeholder2 = 0;
  imu.getEulerRad(&placeholder, &placeholder2, &yawOffset);
  while (run) {

    startTime = millis(); // Get the current time

    control(yawOffset);

    endTime = millis(); // Get the current time again
  
    /* Serial.print("Execution time: ");
    Serial.print(endTime - startTime);
    Serial.println(" ms"); */

    vTaskDelay(pdMS_TO_TICKS(STEP_TIME)); // Delay for STEP_TIME milliseconds
  }
  motorValues.omega_1 = 0;
  motorValues.omega_2 = 0;
  motorValues.omega_3 = 0;
  motorValues.alpha_1 = 0;
  motorValues.alpha_2 = 0;
  motorValues.alpha_3 = 0;
  updateMotor(motorValues);
  vTaskDelete(NULL);
}

void batteryTask(void *pvParameters) {
  while (1) {
    if (getBatteryVoltage() < 8.7){
      Serial.println("Battery low, shutting down");
      //run = false;
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10000 milliseconds aka 10 seconds
  }
  vTaskDelete(NULL);
}

void setup() {


  Wire.begin();

  imu.init();


  delay(1000);
  Serial.begin(115200);

  WiFi.config(local_ip, gateway, subnet);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Trying to connect to wifi...");
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


  const int controlCore = 1;
  const int comCore = 0;

  // add delay to allow the ESCs to initialize
  for (int i = 0; i < 1000; i++) {
    imu.update_IMU();
    delay(10); // Delay for 10 milliseconds
  }

  xTaskCreatePinnedToCore(
    comTask,          // Task function
    "Communication", // Name of task
    10000,            // Stack size of task
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL,             // Task handle.
    comCore                 // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    controlTask,     // Task function
    "Control",       // Name of task
    10000,           // Stack size of task
    NULL,            // Task input parameter
    10,               // Priority of the task
    NULL,            // Task handle.
    controlCore                // Core where the task should run
  );

  xTaskCreatePinnedToCore(
    batteryTask,     // Task function
    "Battery",       // Name of task
    1000,           // Stack size of task
    NULL,            // Task input parameter
    1,               // Priority of the task
    NULL,            // Task handle.
    comCore                // Core where the task should run
  );
  Serial.println("Setup done");
}

void loop() {
  // Empty, since we are using FreeRTOS tasks
}
