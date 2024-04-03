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


const double STEP_TIME = 10; // timeInterval for flight controller
//const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
//const double ALPHA = 0.8; // alpha value for the complementary filter

extern IMU imu;
Servo esc1;
Servo esc2;
Servo esc3;

motorData moterValues;


double x = 0;
double y = 0;
double emils_z = 0;




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
      if (run == false){
        run = true;
      }
      break;
    case 'l':
      run = false;
      break;
    default:
      break;
  }
}


FlightController flightController(STEP_TIME);

void updateMotor(motorData moterValues) {
  
  //make sure the value is max 180
  moterValues.omega_1 = std::min(moterValues.omega_1 / 7500 * 180, 180.0);
  moterValues.omega_2 = std::min(moterValues.omega_2 / 7500 * 180, 180.0);
  moterValues.omega_3 = std::min(moterValues.omega_3 / 7500 * 180, 180.0);



  esc1.write(moterValues.omega_1);
  esc2.write(moterValues.omega_2);
  esc3.write(moterValues.omega_3);
}



void control(){
  //flightController.calculate();
  //motorData data = flightController.getMotorData();
  updateMotor(flightController.calculate());
}

void com(){
  for (int i = 0; i < maxClients; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i] = server.available();
      if (clients[i]) {
        Serial.println("New client connected");
        clients[i].println("Welcome to the server!");
      }
    }
  }
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
  
  clients[0].print("x: "); clients[0].print(pos[0]); clients[0].print(" y: "); clients[0].print(pos[1]); clients[0].print("z: "); clients[0].println(pos[2]);
  clients[1].print("x: "); clients[1].print(pos[0]); clients[1].print(" y: "); clients[1].print(pos[1]); clients[1].print("z: "); clients[1].println(pos[2]);
  clients[2].print("x: "); clients[2].print(pos[0]); clients[2].print(" y: "); clients[2].print(pos[1]); clients[2].print("z: "); clients[2].println(pos[2]);
  clients[3].print("x: "); clients[3].print(pos[0]); clients[3].print(" y: "); clients[3].print(pos[1]); clients[3].print("z: "); clients[3].println(pos[2]);
  clients[4].print("x: "); clients[4].print(pos[0]); clients[4].print(" y: "); clients[4].print(pos[1]); clients[4].print("z: "); clients[4].println(pos[2]);
  //Serial.print(x);
  
  
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

int rhod_test = 0;
unsigned long startTime;
unsigned long endTime;

void controlTask(void *pvParameters) {
  while (1) {
    // Your control task code here
    // This will run indefinitely
    //Serial.print("Control task");
    startTime = millis(); // Get the current time

    control();

    endTime = millis(); // Get the current time again
  
    Serial.print("Execution time: ");
    Serial.print(endTime - startTime);
    Serial.println(" ms");
    if ((run == false)){
      moterValues.omega_1 = 0;
      moterValues.omega_2 = 0;
      moterValues.omega_3 = 0;
      moterValues.alpha = 0;
      updateMotor(moterValues);
      vTaskDelete(NULL);
    }
    //rhod_test++;
    vTaskDelay(pdMS_TO_TICKS(STEP_TIME)); // Delay for STEP_TIME milliseconds
  }
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

  imu.init_IMU();


  delay(1000);
  Serial.begin(115200);

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


  const int controlCore = 1;
  const int comCore = 0;

  // add delay to allow the ESCs to initialize
  delay(10000);

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
