#include <Arduino.h>
#include <FlightController.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <IPAddress.h>
#include "freertos/task.h"



bool run = false;
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


FlightController flightController(STEP_TIME);
IMU& imu = flightController.imu;

double procentPower = 0.5;
double yawOffset = 0;

double x = 0;
double y = 0;
double emils_z = 0;
bool kill_switch = false;



const int maxClients = 5; // Maximum number of clients the server can handle
WiFiClient clients[maxClients]; // Array to hold the client instances

void processCommand(const char command) {
  switch (command) {
    case 'w':
      //change the first value in the vector
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




void updateMotor(motorData motorValues,double powerUp, int state = 1) {

  /* //make sure the value is max 180
  // SLET NÅR VI INDFØRE PWM TIL THRUST
  motorValues.omega_1 = std::min((motorValues.omega_1 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_2 = std::min((motorValues.omega_2 - 182.69 ) / 843.09 * 180, 90.0);
  motorValues.omega_3 = std::min((motorValues.omega_3 - 182.69 ) / 843.09 * 180, 90.0); */

  //make a switch statement for the different states
  switch(state){
    case 0:
      motorValues.omega_1 = 0.0;
      motorValues.omega_2 = 0.0;
      motorValues.omega_3 = 0.0;
      motorValues.alpha_1 = 90.0;
      motorValues.alpha_2 = 90.0;
      motorValues.alpha_3 = 90.0;
      break;
    case 1:
      motorValues.omega_1 = constrain(std::sqrt(4.636e-10*motorValues.omega_1*motorValues.omega_1+0.0007924*motorValues.omega_1),0,120);
      motorValues.omega_2 = constrain(std::sqrt(4.329e-10*motorValues.omega_2*motorValues.omega_2+0.0007126*motorValues.omega_2),0,120);
      motorValues.omega_3 = constrain(std::sqrt(4.304e-10*motorValues.omega_3*motorValues.omega_3+0.0008196*motorValues.omega_3),0,120);

      motorValues.alpha_1 = constrain(motorValues.alpha_1*180/M_PI + 90, 30, 150);
      motorValues.alpha_2 = constrain(motorValues.alpha_2*180/M_PI + 90, 30, 150);
      motorValues.alpha_3 = constrain(motorValues.alpha_3*180/M_PI + 90, 30, 150);
      break;
    case 2:
      motorValues.omega_1 = 20; //20
      motorValues.omega_2 = 20;
      motorValues.omega_3 = 20;
      motorValues.alpha_1 = 90;
      motorValues.alpha_2 = 90;
      motorValues.alpha_3 = 90;
      break;
    case 3:
      motorValues.omega_1 = std::max(constrain(std::sqrt(4.636e-10*motorValues.omega_1*motorValues.omega_1+0.0007924*motorValues.omega_1),0,120)*powerUp,20.0);
      motorValues.omega_2 = std::max(constrain(std::sqrt(4.329e-10*motorValues.omega_2*motorValues.omega_2+0.0007126*motorValues.omega_2),0,120)*powerUp,20.0);
      motorValues.omega_3 = std::max(constrain(std::sqrt(4.304e-10*motorValues.omega_3*motorValues.omega_3+0.0008196*motorValues.omega_3),0,120)*powerUp,20.0);

      motorValues.alpha_1 = constrain(motorValues.alpha_1*180/M_PI + 90, 30, 150);
      motorValues.alpha_2 = constrain(motorValues.alpha_2*180/M_PI + 90, 30, 150);
      motorValues.alpha_3 = constrain(motorValues.alpha_3*180/M_PI + 90, 30, 150);
      break;
  }

  /* Serial.print("omega1: ");
  Serial.print(motorValues.omega_1);
  Serial.print(" omega2: ");
  Serial.print(motorValues.omega_2);
  Serial.print(" omega3: ");
  Serial.println(motorValues.omega_3); */


  esc1.write(motorValues.omega_1*procentPower);
  esc2.write(motorValues.omega_2*procentPower);
  esc3.write(motorValues.omega_3*procentPower);

  servo1.write(ceil(motorValues.alpha_1));
  servo2.write(ceil(motorValues.alpha_2));
  servo3.write(ceil(motorValues.alpha_3));
 


}



void control(double yawOffset,double powerUp, int state){

  updateMotor(flightController.calculate(double(yawOffset)),powerUp,state);
}

double voltage;
void com(){
  bool anyClientConnected = false;  // Flag to track if any client is connected

  for (int i = 0; i < maxClients; i++) {
    if (!clients[i] || !clients[i].connected()) {
      clients[i] = server.available();
      if (clients[i]) {
        //Serial.println("New client connected");
        //clients[i].println("Welcome to the server!");
      }
    } else {
      anyClientConnected = true;  // Set the flag if any client is connected
    }
  }

  // Update run variable based on client connection status
  run = anyClientConnected;
  //run = true;

  // Handle client messages
  for (int i = 0; i < maxClients; i++) {
    if (clients[i] && clients[i].connected() && clients[i].available()) {
      String message = clients[i].readStringUntil('\n');
      //Serial.print("Client ");
      //Serial.print(i);
      //Serial.print(" sent: ");
      //Serial.println(message);
      char c = message[0];
      if (sizeof(c)  == 1){
          processCommand(c); // Process the received command
      }
    }
  }
  //int pos[3]={x,y,emils_z};
  
  double roll = 0;
  double pitch = 0;
  double yaw = 0;
  imu.getEulerRad(&roll, &pitch, &yaw);

  float m1, m2, m3;
  float a1, a2, a3;
  float g1, g2, g3;
  imu.getMagnetom(&m1, &m2, &m3);
  imu.getAcc(&a1, &a2, &a3);
  imu.getGyro(&g1, &g2, &g3);
  Eigen::Quaterniond q;
  imu.getQuaternians(&q.w(), &q.x(), &q.y(), &q.z());
    Serial.print(q.w());
    Serial.print(",");
    Serial.print(q.x());
    Serial.print(",");
    Serial.print(q.y());
    Serial.print(",");
    Serial.println(q.z());
  //send the mag data to all clients
  for (int i = 0; i < maxClients; i++) {
  if (clients[i] && clients[i].connected()) {
    clients[i].print("battery: ");
    clients[i].print(voltage);
    clients[i].print("V ");
    clients[i].print(", roll: ");
    clients[i].println(roll*180/M_PI);
    clients[i].print(", pitch: ");
    clients[i].println(pitch*180/M_PI);
    clients[i].print(", yaw: ");
    clients[i].println(yaw*180/M_PI);


    


    /* clients[i].print(", a2: ");
    clients[i].print(a2);
    clients[i].print(", a3: ");
    clients[i].print(a3);
    clients[i].print(", g1: ");
    clients[i].print(g1);
    clients[i].print(", g2: ");
    clients[i].print(g2);
    clients[i].print(", g3: ");
    clients[i].println(g3); */

  }
}
  

  /* clients[0].print("x: "); clients[0].print(voltage); clients[0].print(" y: "); clients[0].print(yaw-yawOffset*180/M_PI); clients[0].print("p: "); clients[0].println(pitch*180/M_PI);
  clients[1].print("x: "); clients[1].print(voltage); clients[1].print(" y: "); clients[1].print(yaw-yawOffset*180/M_PI); clients[1].print("p: "); clients[1].println(pitch*180/M_PI);
  clients[2].print("x: "); clients[2].print(voltage); clients[2].print(" y: "); clients[2].print(yaw-yawOffset*180/M_PI); clients[2].print("p: "); clients[2].println(pitch*180/M_PI);
  clients[3].print("x: "); clients[3].print(voltage); clients[3].print(" y: "); clients[3].print(yaw-yawOffset*180/M_PI); clients[3].print("p: "); clients[3].println(pitch*180/M_PI);
  clients[4].print("x: "); clients[4].print(voltage); clients[4].print(" y: "); clients[4].print(yaw-yawOffset*180/M_PI); clients[4].print("p: "); clients[4].println(pitch*180/M_PI);
   */
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
  bool start = 0;
  //double powerUp = 0;
  unsigned long startTime;
  unsigned long endTime;
  double placeholder = 0;
  double placeholder2 = 0;

  while (true){

  while (0/* run and !kill_switch */) {
    //updateMotor(motorValues,0,2);
    vTaskDelay(pdMS_TO_TICKS(STEP_TIME));
    if (start == 0){
      updateMotor(motorValues,0,2); //wait 2 seconds but still update the imu
      for (int i = 0; i < 4000/STEP_TIME; i++) {
        vTaskDelay(pdMS_TO_TICKS(STEP_TIME));
        imu.update_IMU();
      }

      imu.getYaw(&yawOffset);
      vTaskDelay(pdMS_TO_TICKS(STEP_TIME));

      for (double powerUp = 0; powerUp < 1; powerUp += 0.005){ {
          if (run and !kill_switch){
          control(yawOffset,powerUp, 3);
          vTaskDelay(pdMS_TO_TICKS(STEP_TIME));
          }
          }
      }
      if (!run or kill_switch){
        break;    
      }
      imu.getYaw(&yawOffset);
      start = 1;
    }
 
    //startTime = millis(); // Get the current time

    control(yawOffset,0, 1); 

    //endTime = millis(); // Get the current time again
  
    /* Serial.print("Execution time: ");
    Serial.print(endTime - startTime);
    Serial.println(" ms"); */

     // Delay for STEP_TIME milliseconds
  }
  imu.update_IMU();
  motorValues.omega_1 = 0.0;
  motorValues.omega_2 = 0.0;
  motorValues.omega_3 = 0.0;
  motorValues.alpha_1 = 0.0;
  motorValues.alpha_2 = 0.0;
  motorValues.alpha_3 = 0.0;
  updateMotor(motorValues,0);
  vTaskDelay(pdMS_TO_TICKS(STEP_TIME));
  start = 0;
  //vTaskDelete(NULL);
  }
}


void batteryTask(void *pvParameters) {
  while (1) {
    if (getBatteryVoltage() < 6.7){
      //Serial.println("Battery low, shutting down");
      kill_switch = true;
    }
    vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10000 milliseconds aka 10 seconds
  }
  vTaskDelete(NULL);
}

void setup() {

  setCpuFrequencyMhz(240);

  Wire.begin();

  imu.init();


  delay(1000);
  Serial.begin(115200);

  WiFi.config(local_ip, gateway, subnet);
  //Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    //Serial.println("Trying to connect to wifi...");
  }
  //Serial.println("Connected to WiFi");

  server.begin();
  //Serial.println("Server started");


  esc1.attach(32, 1000, 2000);
  esc2.attach(25, 1000, 2000);
  esc3.attach(33, 1000, 2000);

  servo1.attach(26);
  servo2.attach(27);
  servo3.attach(14);


  esc1.write(0);
  esc2.write(0);
  esc3.write(0);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);


  /* delay(12000);
  esc1.write(20);
  delay(2000);
  esc2.write(20);
  esc1.write(0);
  delay(2000);
  esc3.write(20);
  esc2.write(0);
  delay(2000);
  esc3.write(0);
  delay(2000);
  servo1.write(115);
  delay(2000);
  servo2.write(115);
  servo1.write(90);
  delay(2000);
  servo3.write(115);
  servo2.write(90);
  delay(2000);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  esc1.write(0);
  esc2.write(0);
  esc3.write(0);
  delay(30000); */


  const int controlCore = 1;
  const int comCore = 0;


  // add delay to allow the ESCs to initialize
  double temp_yaw = 0;
  for (int i = 0; i < (12*1000)/STEP_TIME; i++) {
    imu.update_IMU();
    imu.getYaw(&temp_yaw);
    //Serial.print("yaw: ");
    //Serial.println(temp_yaw);
    delay(STEP_TIME); // Delay for 5 milliseconds
  }

  xTaskCreatePinnedToCore(
    comTask,          // Task function
    "Communication", // Name of task
    100000,            // Stack size of task
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
  //Serial.println("Setup done");
}

void loop() {
  // Empty, since we are using FreeRTOS tasks
}
