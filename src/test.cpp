#include <Arduino.h>
#include <IMU.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <IPAddress.h>

// Constants
const double STEP_TIME = 20; // timeInterval
const char* ssid = "TDC-C0A4"; // Your WiFi SSID
const char* password = "3356f79c4"; // Your WiFi password
const int port = 80;
int counter = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;
double z1 = 0;


// class for server
WiFiServer server(port);

IPAddress local_IP(192, 168, 1, 50);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

IMU imu;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    // Initialize the IMU
    imu.init_sensors();

    // Connect to Wi-Fi
    if (!WiFi.config(local_IP, gateway, subnet)){}
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    }

    // Start the server
    server.begin();
}

void loop() {
  
  WiFiClient client = server.available(); // Check if a client has connected

  if (client) {
    while (client.connected()) {
    
      //Get roll, pitch and yaw angle and alttitude from ToF sensor
      imu.getIMUData(&roll, &pitch, &yaw, &z1);

      /*Serial.print("X:"); Serial.print(angle.roll);
      Serial.print("Y:"); Serial.print(angle.pitch);
      Serial.print("Z:"); Serial.print(angle.yaw);
      Serial.print("z1:"); Serial.println(angle.z1);*/

      //Send message
      if (counter % 5 ==0.0){
        client.print("X:"); 
        client.print(roll); client.print("\t");
        client.print("Y:"); 
        client.print(pitch); client.print("\t");
        client.print("Z:"); 
        client.print(yaw); client.print("\t");
        client.print("z1:");
        client.print(z1); client.print("\t");
        client.print("n");
      }

      counter += 1;
      delay(STEP_TIME);
    }
  }
}
