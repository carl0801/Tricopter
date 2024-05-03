#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

BLEServer* pServer;

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


double targetPos[3] = {0, 0, 0};


// Create BLE service
BLEService* createService(const char* serviceUUID) {
  return pServer->createService(serviceUUID);
}

// Create BLE characteristic with Read, Write and Notify properties
BLECharacteristic* createCharacteristicRWN(BLEService* pService, const char* characteristicUUID) {
  return pService->createCharacteristic(
             characteristicUUID,
             BLECharacteristic::PROPERTY_READ |
             BLECharacteristic::PROPERTY_WRITE |
             BLECharacteristic::PROPERTY_NOTIFY
           );
}

// Create BLE characteristic with Read and Notify properties
BLECharacteristic* createCharacteristicRN(BLEService* pService, const char* characteristicUUID) {
  return pService->createCharacteristic(
             characteristicUUID,
             BLECharacteristic::PROPERTY_READ |
             BLECharacteristic::PROPERTY_NOTIFY
           );
}


// Callback for characteristic write:
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    // Get value length
    int valueLength = pCharacteristic->getValue().length();

    // Check if value length is valid for 3 floats
    if (valueLength != sizeof(float) * 3 + 1) {
      Serial.println("Invalid data length for 3 floats format");
      return;
    }

    // Temporary variables to store received values as doubles
    double value1, value2, value3;

    // Extract and convert 3 floats to doubles
    value1 = *((double*)&pCharacteristic->getValue()[1]);  // Type casting to double
    value2 = *((double*)&pCharacteristic->getValue()[5]);  // Type casting to double
    value3 = *((double*)&pCharacteristic->getValue()[9]);  // Type casting to double

    // Get characteristic UUID
    std::string uuid = pCharacteristic->getUUID().toString();
    // Update variables based on characteristic UUID (implement your logic here)
    if (uuid == GOAL_POSITION_CHARACTERISTIC_UUID) {
      // Update goal position variables
      targetPos[0] = value1;
      targetPos[1] = value2;
      targetPos[2] = value3;
    } //else if (uuid == GOAL_ORIENTATION_CHARACTERISTIC_UUID) {
      // Update goal orientation variables

    //}// else if (uuid == SERVO_ANGLE_CHARACTERISTIC_UUID) {
      // Update servo angle variables

    //}// else if (uuid == MOTOR_PWM_CHARACTERISTIC_UUID) {
      // Update pwm variables

    //}
    else {
      Serial.println("Unknown characteristic UUID");
    }
  }
};

// Callback for client disconnect
class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer* pServer) {
        pServer->startAdvertising();
    }
};



// void setupBLE() {
//   BLEDevice::init("Tricopter_FlightController");

//   pServer = BLEDevice::createServer();

//   // Create Services
//   BLEService* navigationService = createService(NAVIGATION_SERVICE_UUID);
//   BLEService* controlService = createService(CONTROL_SERVICE_UUID);
//   BLEService* sensorService = createService(SENSOR_SERVICE_UUID);

//   // Navigation Service Characteristics
//   BLECharacteristic* positionCharacteristic = createCharacteristicRN(navigationService, POSITION_CHARACTERISTIC_UUID);
//   BLECharacteristic* orientationCharacteristic = createCharacteristicRN(navigationService, ORIENTATION_CHARACTERISTIC_UUID);
//   BLECharacteristic* linearAccelerationCharacteristic = createCharacteristicRN(navigationService, LINEAR_ACCELERATION_CHARACTERISTIC_UUID);
//   BLECharacteristic* angularAccelerationCharacteristic = createCharacteristicRN(navigationService, ANGULAR_ACCELERATION_CHARACTERISTIC_UUID);
//   BLECharacteristic* goalPositionCharacteristic = createCharacteristicRWN(navigationService, GOAL_POSITION_CHARACTERISTIC_UUID);
//   BLECharacteristic* goalOrientationCharacteristic = createCharacteristicRWN(navigationService, GOAL_ORIENTATION_CHARACTERISTIC_UUID);

//   // Control Service Characteristics
//   BLECharacteristic* servoAngleCharacteristic = createCharacteristicRWN(controlService, SERVO_ANGLE_CHARACTERISTIC_UUID);
//   BLECharacteristic* motorPWMCharacteristic = createCharacteristicRWN(controlService, MOTOR_PWM_CHARACTERISTIC_UUID);

//   // Sensor Service Characteristics
//   BLECharacteristic* lidarZCharacteristic = createCharacteristicRN(sensorService, LIDAR_Z_CHARACTERISTIC_UUID);
//   BLECharacteristic* lidarXYCharacteristic = createCharacteristicRN(sensorService, LIDAR_XY_CHARACTERISTIC_UUID);
//   BLECharacteristic* timeCharacteristic = createCharacteristicRN(sensorService, TIME_CHARACTERISTIC_UUID);
//   BLECharacteristic* batteryCharacteristic = createCharacteristicRN(sensorService, BATTERY_CHARACTERISTIC_UUID);
//   BLECharacteristic* powerUsageCharacteristic = createCharacteristicRN(sensorService, POWER_USAGE_CHARACTERISTIC_UUID);

//   // Set callback for characteristic write (implement the logic in CharacteristicCallbacks class)
//   CharacteristicCallbacks characteristicCallback;
//   goalPositionCharacteristic->setCallbacks(&characteristicCallback);
//   goalOrientationCharacteristic->setCallbacks(&characteristicCallback);
//   servoAngleCharacteristic->setCallbacks(&characteristicCallback);
//   motorPWMCharacteristic->setCallbacks(&characteristicCallback);

//   // Start advertising the server
//   pServer->setCallbacks(new ServerCallbacks());
//   pServer->startAdvertising();
//   Serial.println("BLE Server started advertising");
// }

// Convert 3 floats into a bytearray and notify over BLE
void notifyFloatBLE(BLECharacteristic* pCharacteristic, const double* data) {
  
  // convert the 3 doubles to 3 floats
  float dataFloat[3];
  dataFloat[0] = (float)data[0];
  dataFloat[1] = (float)data[1];
  dataFloat[2] = (float)data[2];
  
  // Create buffer to store data
  uint8_t buffer[sizeof(float) * 3];

  // Copy data to buffer
  memcpy(&buffer[0], dataFloat, sizeof(float) * 3);

  // Notify characteristic
  pCharacteristic->setValue(buffer, sizeof(float) * 3);
  pCharacteristic->notify();
}


void notifyDoubleBLE(BLECharacteristic* pCharacteristic, const float* data) {
  // Create buffer to store data
  uint8_t buffer[sizeof(double)];

  // Copy data to buffer
  memcpy(&buffer[0], data, sizeof(double));

  // Notify characteristic
  pCharacteristic->setValue(buffer, sizeof(double));
  pCharacteristic->notify();
}