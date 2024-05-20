#include <Arduino.h>
#include <FlightControllerMPC.hpp>
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
  Serial.println(moterValues.force_1);
  Serial.println(moterValues.force_2);
  Serial.println(moterValues.force_3);
  Serial.println(moterValues.tilt);
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

void comTask(void *pvParameters) {
  while (1) {
    com();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
  }
  vTaskDelete(NULL);
}

void controlTask(void *pvParameters) {
  while (1) {
    control();

    vTaskDelay(pdMS_TO_TICKS(STEP_TIME)); // Delay for STEP_TIME milliseconds
  }
  vTaskDelete(NULL);
}

void setup() {
  Wire.begin();

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

  // add delay to allow the ESCs to initialize
  delay(10000);

  xTaskCreate(
    comTask,          // Task function
    "Communication", // Name of task
    10000,            // Stack size of task
    NULL,             // Task input parameter
    1,                // Priority of the task
    NULL             // Task handle.
  );

  xTaskCreate(
    controlTask,     // Task function
    "Control",       // Name of task
    10000,           // Stack size of task
    NULL,            // Task input parameter
    10,               // Priority of the task
    NULL            // Task handle.
  );
  Serial.println("Setup done");
}

void loop() {
  // Empty, since we are using FreeRTOS tasks
}
