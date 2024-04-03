#include "BLE_COMMS.h"

BLEServer* pServer;

std::vector<const char*> uuids = readUUIDsFromFile("src/v4_uuids.txt");

#define POSE_SERVICE_UUID       uuids[0]
#define ACCELERATION_SERVICE_UUID uuids[1]
#define LIDAR_SERVICE_UUID      uuids[2]
#define SERVO_SERVICE_UUID      uuids[3]
#define MOTORS_SERVICE_UUID     uuids[4]
#define UTILITIES_SERVICE_UUID  uuids[5]

#define X_CHARACTERISTIC_UUID   uuids[6]
#define Y_CHARACTERISTIC_UUID   uuids[7]
#define Z_CHARACTERISTIC_UUID   uuids[8]
#define QUATERNION_A_CHARACTERISTIC_UUID   uuids[9]
#define QUATERNION_B_CHARACTERISTIC_UUID   uuids[10]
#define QUATERNION_C_CHARACTERISTIC_UUID   uuids[11]

#define X_ACCELERATION_CHARACTERISTIC_UUID   uuids[12]
#define Y_ACCELERATION_CHARACTERISTIC_UUID   uuids[13]
#define Z_ACCELERATION_CHARACTERISTIC_UUID   uuids[14]
#define QUATERNION_A_ACCELERATION_CHARACTERISTIC_UUID   uuids[15]
#define QUATERNION_B_ACCELERATION_CHARACTERISTIC_UUID   uuids[16]
#define QUATERNION_C_ACCELERATION_CHARACTERISTIC_UUID   uuids[17]

#define DISTANCE_Z_CHARACTERISTIC_UUID   uuids[18]
#define DISTANCE_XY_CHARACTERISTIC_UUID   uuids[19]

#define ANGLE_A_CHARACTERISTIC_UUID   uuids[20]
#define ANGLE_B_CHARACTERISTIC_UUID   uuids[21]
#define ANGLE_C_CHARACTERISTIC_UUID   uuids[22]

#define PWM_A_CHARACTERISTIC_UUID   uuids[23]
#define PWM_B_CHARACTERISTIC_UUID   uuids[24]
#define PWM_C_CHARACTERISTIC_UUID   uuids[25]

#define TIME_CHARACTERISTIC_UUID   uuids[26]
#define BATTERY_CHARACTERISTIC_UUID   uuids[27]



// Create BLE service
BLEService* createService(const char* serviceUUID) {
  return pServer->createService(serviceUUID);
}

// Create BLE characteristic
BLECharacteristic* createCharacteristic(BLEService* pService, const char* characteristicUUID) {
  return pService->createCharacteristic(
             characteristicUUID,
             BLECharacteristic::PROPERTY_READ |
             BLECharacteristic::PROPERTY_WRITE |
             BLECharacteristic::PROPERTY_NOTIFY
           );
}

// Callback for characteristic write
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        Serial.print("Characteristic value: ");
        Serial.println(value.c_str());
        // Handle the received value here
    }
};

// Callback for client disconnect
class ServerCallbacks : public BLEServerCallbacks {
    void onDisconnect(BLEServer* pServer) {
        Serial.println("Client disconnected");
        pServer->startAdvertising();
    }
};



void setup() {
  Serial.begin(115200);

  BLEDevice::init("Tricopter_FlightController");

  pServer = BLEDevice::createServer();

  // Create services
  BLEService *poseService = createService(POSE_SERVICE_UUID);
  BLEService *accelerationService = createService(ACCELERATION_SERVICE_UUID);
  BLEService *lidarService = createService(LIDAR_SERVICE_UUID);
  BLEService *servoService = createService(SERVO_SERVICE_UUID);
  BLEService *motorsService = createService(MOTORS_SERVICE_UUID);
  BLEService *utilitiesService = createService(UTILITIES_SERVICE_UUID);

  // Create characteristics
  BLECharacteristic *xCharacteristic = createCharacteristic(poseService, X_CHARACTERISTIC_UUID);
  BLECharacteristic *yCharacteristic = createCharacteristic(poseService, Y_CHARACTERISTIC_UUID);
  BLECharacteristic *zCharacteristic = createCharacteristic(poseService, Z_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionACharacteristic = createCharacteristic(poseService, QUATERNION_A_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionBCharacteristic = createCharacteristic(poseService, QUATERNION_B_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionCCharacteristic = createCharacteristic(poseService, QUATERNION_C_CHARACTERISTIC_UUID);

  BLECharacteristic *xAccelerationCharacteristic = createCharacteristic(accelerationService, X_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic *yAccelerationCharacteristic = createCharacteristic(accelerationService, Y_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic *zAccelerationCharacteristic = createCharacteristic(accelerationService, Z_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionAAccelerationCharacteristic = createCharacteristic(accelerationService, QUATERNION_A_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionBAccelerationCharacteristic = createCharacteristic(accelerationService, QUATERNION_B_ACCELERATION_CHARACTERISTIC_UUID);
  BLECharacteristic *quaternionCAccelerationCharacteristic = createCharacteristic(accelerationService, QUATERNION_C_ACCELERATION_CHARACTERISTIC_UUID);

  BLECharacteristic *distanceZCharacteristic = createCharacteristic(lidarService, DISTANCE_Z_CHARACTERISTIC_UUID);
  BLECharacteristic *distanceXYCharacteristic = createCharacteristic(lidarService, DISTANCE_XY_CHARACTERISTIC_UUID);

  BLECharacteristic *angleACharacteristic = createCharacteristic(servoService, ANGLE_A_CHARACTERISTIC_UUID);
  BLECharacteristic *angleBCharacteristic = createCharacteristic(servoService, ANGLE_B_CHARACTERISTIC_UUID);
  BLECharacteristic *angleCCharacteristic = createCharacteristic(servoService, ANGLE_C_CHARACTERISTIC_UUID);

  BLECharacteristic *pwmACharacteristic = createCharacteristic(motorsService, PWM_A_CHARACTERISTIC_UUID);
  BLECharacteristic *pwmBCharacteristic = createCharacteristic(motorsService, PWM_B_CHARACTERISTIC_UUID);
  BLECharacteristic *pwmCCharacteristic = createCharacteristic(motorsService, PWM_C_CHARACTERISTIC_UUID);

  BLECharacteristic *timeCharacteristic = createCharacteristic(utilitiesService, TIME_CHARACTERISTIC_UUID);
  BLECharacteristic *batteryCharacteristic = createCharacteristic(utilitiesService, BATTERY_CHARACTERISTIC_UUID);

  // Advertise services
  poseService->start();
  accelerationService->start();
  lidarService->start();
  servoService->start();
  motorsService->start();
  utilitiesService->start();

  // Advertise device
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void loop() {
  // Nothing to do here for now
}