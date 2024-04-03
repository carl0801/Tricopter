#ifndef BLE_COMMS_H
#define BLE_COMMS_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for the services and characteristics
#include <fstream>
#include <vector>
#include <string>

std::vector<const char*> readUUIDsFromFile(const std::string& filePath) {
    std::vector<const char*> uuids;
    std::ifstream file(filePath);
    std::string line;
    while (std::getline(file, line)) {
        uuids.push_back(line.c_str());
    }
    return uuids;
}













#endif