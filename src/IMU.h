#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "MPU6050.h"
#include "HMC5883L.h"

class IMU {
    private:
        // Constants
        const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
        const double ALPHA = 0.8; // alpha value for the complementary filter
    public:
        // Structures
        struct RawData {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            int16_t mx, my, mz;
        };

        struct Angle {
            double z1 = 0;
            double z2 = 0;
            double roll = 0;
            double pitch = 0;
            double yaw = 0;
        };

        struct Mag {
            double x = 0;
            double y = 0;
            double z = 0;
            double calX = 0;
            double calY = 0;
            double calZ = 0;
        };

        // Functions
        void init_sensors();
        void getIMUData(double *roll, double *pitch, double *yaw, double *z1);
};

#endif // MY_SENSORS_H
