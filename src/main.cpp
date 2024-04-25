#include <Arduino.h>
#include <FlightController.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <IPAddress.h>
#include "freertos/task.h"

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

Servo servo1;
Servo servo2;
Servo servo3;

motorData motorValues;


double x = 0;
double y = 0;
double emils_z = 0;
bool kill_switch = false;



const int maxClients = 5; // Maximum number of clients the server can handle
WiFiClient clients[maxClients]; // Array to hold the client instances

void processCommand(const char command) {
  switch (command) {
    case 'w':
      y += 5;
      //updateESC1(y);
      break;
    case 's':
      y -= 5;
      //updateESC1(y);
      break;
    case 'a':
      x -= 5;
      //updateESC2(x);
      break;
    case 'd':
      x += 5;
      //updateESC2(x);
      break;
    case 'r':
      emils_z += 0.1;
      //updateESC3(z);
      break;
    case 'f':
      emils_z -= 0.1;
      //updateESC3(z);
      break;
    case 'm':
        run = true;
        kill_switch = false;
      break;
    case 'l':
      run = false;
      kill_switch = true;
      break;
    default:
      break;
  }
}


FlightController flightController(STEP_TIME);
IMU& imu = flightController.imu;
void updateMotor(motorData motorValues) {

  //make sure the value is max 180
  motorValues.omega_1 = std::min((motorValues.omega_1 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_2 = std::min((motorValues.omega_2 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_3 = std::min((motorValues.omega_3 - 182.69 ) / 843.09 * 180, 90.0);

  //motorValues.alpha_1 = 0.0;
  //motorValues.alpha_2 = 0.0;
  //motorValues.alpha_3 = 0.0;

  //constrain the value to be between 0 and 180
  motorValues.alpha_1 = constrain(motorValues.alpha_1*180/M_PI + 90, 30, 150);
  motorValues.alpha_2 = constrain(motorValues.alpha_2*180/M_PI + 90, 30, 150);
  motorValues.alpha_3 = constrain(motorValues.alpha_3*180/M_PI + 90, 30, 150);


  /* motorValues.alpha_1 = map(motorValues.alpha_1, 0, 180, 500, 2500);
  motorValues.alpha_2 = map(motorValues.alpha_2, 0, 180, 500, 2500);
  motorValues.alpha_3 = map(motorValues.alpha_3, 0, 180, 500, 2500); */


  esc1.write(motorValues.omega_1*0.7);
  esc2.write(motorValues.omega_2*0.7);
  esc3.write(motorValues.omega_3*0.7);

  
  /* servo1.writeMicroseconds(motorValues.alpha_1);
  servo2.writeMicroseconds(motorValues.alpha_2);
  servo3.writeMicroseconds(motorValues.alpha_3); */

  servo1.write(motorValues.alpha_1);
  servo2.write(motorValues.alpha_2);
  servo3.write(motorValues.alpha_3);
 


}



void control(double yawOffset){
  //flightController.calculate();
  //motorData data = flightController.getMotorData();
  updateMotor(flightController.calculate(double(yawOffset)));
}
double voltage;
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
  int pos[3]={x,y,emils_z};
  
  clients[0].print("x: "); clients[0].print(voltage); clients[0].print(" y: "); clients[0].print(pos[1]); clients[0].print("z: "); clients[0].println(pos[2]);
  clients[1].print("x: "); clients[1].print(voltage); clients[1].print(" y: "); clients[1].print(pos[1]); clients[1].print("z: "); clients[1].println(pos[2]);
  clients[2].print("x: "); clients[2].print(voltage); clients[2].print(" y: "); clients[2].print(pos[1]); clients[2].print("z: "); clients[2].println(pos[2]);
  clients[3].print("x: "); clients[3].print(voltage); clients[3].print(" y: "); clients[3].print(pos[1]); clients[3].print("z: "); clients[3].println(pos[2]);
  clients[4].print("x: "); clients[4].print(pos[0]); clients[4].print(" y: "); clients[4].print(pos[1]); clients[4].print("z: "); clients[4].println(pos[2]);
  //Serial.print(x);
  
  
}


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
  while (true){
  unsigned long startTime;
  unsigned long endTime;
  double yawOffset = 0;
  double placeholder = 0;
  double placeholder2 = 0;
  imu.getEulerRad(&placeholder, &placeholder2, &yawOffset);
  while (run and !kill_switch) {

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
  vTaskDelay(pdMS_TO_TICKS(STEP_TIME));
  //vTaskDelete(NULL);
  }
}

void batteryTask(void *pvParameters) {
  while (1) {
    if (getBatteryVoltage() < 6.7){
      Serial.println("Battery low, shutting down");
      kill_switch = true;
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

  servo1.attach(14);
  servo2.attach(27);
  servo3.attach(26);


  esc1.write(0);
  esc2.write(0);
  esc3.write(0);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);


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
